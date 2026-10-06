if(CONFIG_ZMK_INPUT_PROCESSOR_SCROLL_INERTIA)
    target_sources_ifdef(CONFIG_ZMK_INPUT_PROCESSOR_SCROLL_INERTIA app PRIVATE src/input_processor_scroll_inertia.c)
    # Toucan Studio: runtime API header
    zephyr_include_directories(include)
endif()
