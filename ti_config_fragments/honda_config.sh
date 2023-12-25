export ARCH=arm
set -x
CONFIG_FRAGMENTS=" \
$KERNEL_ROOT/arch/arm/configs/omap2plus_defconfig \
$KERNEL_ROOT/ti_config_fragments/dra7_only.cfg \
$KERNEL_ROOT/ti_config_fragments/baseport.cfg \
$KERNEL_ROOT/ti_config_fragments/power.cfg \
$KERNEL_ROOT/ti_config_fragments/connectivity.cfg \
$KERNEL_ROOT/ti_config_fragments/ipc.cfg \
$KERNEL_ROOT/ti_config_fragments/audio_display.cfg \
$KERNEL_ROOT/ti_config_fragments/radio.cfg \
$KERNEL_ROOT/ti_config_fragments/wlan.cfg \
$KERNEL_ROOT/ti_config_fragments/auto.cfg \
$KERNEL_ROOT/ti_config_fragments/android_omap.cfg \
$KERNEL_ROOT/ti_config_fragments/honda_common.cfg \
$KERNEL_ROOT/ti_config_fragments/$TARGET_PRODUCT.cfg "

echo "TESTING SPECTRUM_DISPLAY FLAG"
if [ "$SPECTRUM_DISPLAY" == "true" ]
then
	CONFIG_FRAGMENTS=" \
$CONFIG_FRAGMENTS \
$KERNEL_ROOT/ti_config_fragments/spectrum_display.cfg"
	echo "Support for SPECTRUM_DISPLAY (CONFIG_DISPLAY_PANEL_TLC59108) added"
fi

echo "TESTING HRALVDS FLAGS"
# If the board has alpine display
if [ "$HRALVDS_THRA_DISPLAY" == "true" ]
then
    CONFIG_FRAGMENTS=" \
$CONFIG_FRAGMENTS \
$KERNEL_ROOT/ti_config_fragments/hralvds_thra_support.cfg "
   echo "Support for HRALVDS_THRA_DISPLAY selected"
elif [ "$HRALVDS_TJBA_DISPLAY" == "true" ]
then
    CONFIG_FRAGMENTS=" \
$CONFIG_FRAGMENTS \
$KERNEL_ROOT/ti_config_fragments/hralvds_tjba_support.cfg "
   echo "Support for HRALVDS_TJBA_DISPLAY selected"
else
   echo "Error: neither HRALVDS_THRA_DISPLAY nor HRALVDS_TJBA_DISPLAY is set"
fi

# For USB debugging
if [ "$BUILD_USBMON" == "true" ]
then
    CONFIG_FRAGMENTS=" \
$CONFIG_FRAGMENTS \
$KERNEL_ROOT/ti_config_fragments/usbdebug_options.cfg "
fi

# For secure boards
if [[ $TARGET_PRODUCT == gen1*_HS || $TARGET_PRODUCT == car_jacinto6evm_hs ]]
then
   CONFIG_FRAGMENTS=" \
$CONFIG_FRAGMENTS \
$KERNEL_ROOT/ti_config_fragments/secure_options.cfg "
fi

# For custom honda boards
if [[ $TARGET_PRODUCT == gen1* ]]
then
   CONFIG_FRAGMENTS=" \
$CONFIG_FRAGMENTS \
$KERNEL_ROOT/ti_config_fragments/honda_options.cfg "
fi


$KERNEL_ROOT/scripts/kconfig/merge_config.sh -m -O $KPATH $CONFIG_FRAGMENTS
