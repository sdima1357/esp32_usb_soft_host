set(srcs "usb_host.c"
    "usb_test.c"
    )


idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS ".")

