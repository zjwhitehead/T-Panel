#define TOUCH_MODULES_CST_MUTUAL
#include "lvgl.h"
#include "Arduino_GFX_Library.h"
#include "pin_config.h"
#include "TouchLib.h"

static bool Touch_Int_Flag = false;

TouchLib touch(Wire, TOUCH_SDA, TOUCH_SCL, CST3240_ADDRESS);

static lv_disp_draw_buf_t draw_buf;
static lv_disp_drv_t disp_drv;
static lv_indev_drv_t indev_drv;

Arduino_DataBus *bus = new Arduino_XL9535SWSPI(IIC_SDA, IIC_SCL, -1, XL95X5_CS, XL95X5_SCLK, XL95X5_MOSI);
Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
    -1, LCD_VSYNC, LCD_HSYNC, LCD_PCLK,
    LCD_B0, LCD_B1, LCD_B2, LCD_B3, LCD_B4,
    LCD_G0, LCD_G1, LCD_G2, LCD_G3, LCD_G4, LCD_G5,
    LCD_R0, LCD_R1, LCD_R2, LCD_R3, LCD_R4,
    1, 20, 2, 0,
    1, 30, 8, 1,
    10, 6000000L, false,
    0, 0);
Arduino_RGB_Display *gfx = new Arduino_RGB_Display(
    LCD_WIDTH, LCD_HEIGHT, rgbpanel, 0, true,
    bus, -1, st7701_type9_init_operations, sizeof(st7701_type9_init_operations));

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
    gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif

    lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    if (Touch_Int_Flag == true)
    {
        Touch_Int_Flag = false;

        touch.read();
        if (touch.getPointNum() > 0)
        {
            TP_Point t = touch.getPoint(0);

            data->state = LV_INDEV_STATE_PR;
            data->point.x = t.x;
            data->point.y = t.y;
        }
        else
        {
            data->state = LV_INDEV_STATE_REL;
        }
    }
    else
    {
        data->state = LV_INDEV_STATE_REL;
    }
}

static void btn_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * btn = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        static int count = 0;
        count++;
        lv_obj_t * label = lv_obj_get_child(btn, 0);
        lv_label_set_text_fmt(label, "Count: %d", count);
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Hello World Setup");

    pinMode(LCD_BL, OUTPUT);
    digitalWrite(LCD_BL, HIGH);

    attachInterrupt(
        TOUCH_INT,
        []
        {
            Touch_Int_Flag = true;
        },
        FALLING);

    Wire.begin(IIC_SDA, IIC_SCL);

    gfx->begin();
    gfx->fillScreen(BLACK);

    gfx->XL_digitalWrite(TOUCH_RST, LOW);
    delay(200);
    gfx->XL_digitalWrite(TOUCH_RST, HIGH);
    delay(200);

    touch.init();

    lv_init();

    lv_color_t *buf_1 = (lv_color_t *)heap_caps_malloc(48 * 1024, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    lv_color_t *buf_2 = (lv_color_t *)heap_caps_malloc(48 * 1024, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    lv_disp_draw_buf_init(&draw_buf, buf_1, buf_2, 48 * 1024 * 2);

    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_WIDTH;
    disp_drv.ver_res = LCD_HEIGHT;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.full_refresh = 1;
    lv_disp_drv_register(&disp_drv);

    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    // Create a label with "Hello World" text
    lv_obj_t * hello_label = lv_label_create(lv_scr_act());
    lv_label_set_text(hello_label, "Hello World");
    lv_obj_align(hello_label, LV_ALIGN_TOP_MID, 0, 10);

    // Create a button
    lv_obj_t * btn = lv_btn_create(lv_scr_act());
    lv_obj_set_size(btn, 120, 50);
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_ALL, NULL);

    // Add label to button
    lv_obj_t * btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Count: 0");
    lv_obj_center(btn_label);
}

void loop()
{
    lv_timer_handler();
    delay(5);
}
