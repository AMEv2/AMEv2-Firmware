idf_component_register(SRCS "cmd_control.c" 
                        "ame_buzzer.c" 
                        "rtttl.c" 
                        "ame_anneal.c" 
                        "ame_dropgate.c" 
                        "ota_task.c" 
                        "ame_cooling.c" 
                        "ame_stepper.c" 
                        "ame_ui.c" 
                        "ame_adc.c" 
                        "ame_temperature.c" 
                        "main.c" 
                        "u8g2_esp32_hal.c"
                        "ame_cfg.c"
                        "cmd_pubsub.c"
                    INCLUDE_DIRS ""
                    REQUIRES driver 
                            console
                            u8g2
                            encoder
                            esp32-owb
                            esp32-ds18b20
                            pubsub-c
                            nvs_flash
                            spi_flash
                            app_update
                            esp_adc
                            esp_http_server
                            esp_wifi
                    EMBED_TXTFILES "web/index.html"
                        "web/favicon.ico")

