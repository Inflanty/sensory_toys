deps_config := \
	/home/JG/Workspace/esp/esp-idf/components/app_trace/Kconfig \
	/home/JG/Workspace/esp/esp-idf/components/aws_iot/Kconfig \
	/home/JG/Workspace/esp/esp-idf/components/bt/Kconfig \
	/home/JG/Workspace/esp/esp-idf/components/esp32/Kconfig \
	/home/JG/Workspace/esp/esp-idf/components/ethernet/Kconfig \
	/home/JG/Workspace/esp/esp-idf/components/fatfs/Kconfig \
	/home/JG/Workspace/esp/esp-idf/components/freertos/Kconfig \
	/home/JG/Workspace/esp/esp-idf/components/heap/Kconfig \
	/home/JG/Workspace/esp/esp-idf/components/log/Kconfig \
	/home/JG/Workspace/esp/esp-idf/components/lwip/Kconfig \
	/home/JG/Workspace/esp/esp-idf/components/mbedtls/Kconfig \
	/home/JG/Workspace/esp/esp-idf/components/openssl/Kconfig \
	/home/JG/Workspace/esp/esp-idf/components/pthread/Kconfig \
	/home/JG/Workspace/esp/esp-idf/components/spi_flash/Kconfig \
	/home/JG/Workspace/esp/esp-idf/components/spiffs/Kconfig \
	/home/JG/Workspace/esp/esp-idf/components/tcpip_adapter/Kconfig \
	/home/JG/Workspace/esp/esp-idf/components/wear_levelling/Kconfig \
	/home/JG/Workspace/esp/esp-idf/components/bootloader/Kconfig.projbuild \
	/home/JG/Workspace/esp/esp-idf/components/esptool_py/Kconfig.projbuild \
	/home/JG/Workspace/esp/esp-idf/components/partition_table/Kconfig.projbuild \
	/home/JG/Workspace/esp/esp-idf/Kconfig

include/config/auto.conf: \
	$(deps_config)


$(deps_config): ;
