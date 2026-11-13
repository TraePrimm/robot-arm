rd() { a=$(arm-none-eabi-nm build/Robot-Arm-FreeRTOS.elf | awk -v s="$1" '$3==s{print "0x"$1}'); printf "%-20s" "$1"; STM32_Programmer_CLI -c port=SWD mode=HOTPLUG -r32 $a 4 2>/dev/null | grep -iE "0x[0-9a-f]{8} *:"; }
fwdump() { for v in g_fault_code g_net_ready g_ros_ping_tries g_ros_fail_line g_ros_fail_rc g_heap_free j0_ready j0_tmc g_ros_cmd_count g_ros_last_cmd_hz; do rd $v; done; }
