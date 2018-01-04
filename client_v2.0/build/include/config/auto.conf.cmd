deps_config := \
	/home/jan/Workspace/esp/esp-idf/components/app_trace/Kconfig \
	/home/jan/Workspace/esp/esp-idf/components/aws_iot/Kconfig \
	/home/jan/Workspace/esp/esp-idf/components/bt/Kconfig \
	/home/jan/Workspace/esp/esp-idf/components/esp32/Kconfig \
	/home/jan/Workspace/esp/esp-idf/components/ethernet/Kconfig \
	/home/jan/Workspace/esp/esp-idf/components/fatfs/Kconfig \
	/home/jan/Workspace/esp/esp-idf/components/freertos/Kconfig \
	/home/jan/Workspace/esp/esp-idf/components/heap/Kconfig \
	/home/jan/Workspace/esp/esp-idf/components/libsodium/Kconfig \
	/home/jan/Workspace/esp/esp-idf/components/log/Kconfig \
	/home/jan/Workspace/esp/esp-idf/components/lwip/Kconfig \
	/home/jan/Workspace/esp/esp-idf/components/mbedtls/Kconfig \
	/home/jan/Workspace/esp/esp-idf/components/openssl/Kconfig \
	/home/jan/Workspace/esp/esp-idf/components/pthread/Kconfig \
	/home/jan/Workspace/esp/esp-idf/components/spi_flash/Kconfig \
	/home/jan/Workspace/esp/esp-idf/components/spiffs/Kconfig \
	/home/jan/Workspace/esp/esp-idf/components/tcpip_adapter/Kconfig \
	/home/jan/Workspace/esp/esp-idf/components/wear_levelling/Kconfig \
	/home/jan/Workspace/esp/esp-idf/components/bootloader/Kconfig.projbuild \
	/home/jan/Workspace/esp/esp-idf/components/esptool_py/Kconfig.projbuild \
	/home/jan/Workspace/esp/esp-idf/components/partition_table/Kconfig.projbuild \
	/home/jan/Workspace/esp/esp-idf/Kconfig

include/config/auto.conf: \
	$(deps_config)


$(deps_config): ;
