# procedures for SWV baudrate

proc extract_uint32 { arr start } {
	set result 0
	for {set i 3} {$i >= 0} {incr i -1} {
		set byte_index [expr {$start + $i}]
		set value [expr "0x[lindex $arr $byte_index]"]
		set result [expr "($result << 8) + $value"]
	}
	return $result
}

proc extract_uint32_hex { arr start } {
	return [format {%x} [extract_uint32 $arr $start]]
}

proc swv { args } {
	global swv_cmd
	global USE_SWO
	global CHIPNAME

	switch -exact [lindex $args 0] {
		start {
			set swv_port [lindex $swv_cmd 3]
			set trace_clk [lindex $swv_cmd 5]
			set baudrate [expr "0x[lindex $args 1] * 1000"]

			if {[expr {($USE_SWO == 1)}]} {
				$CHIPNAME.swo configure -protocol uart -output $swv_port -traceclk $trace_clk -pin-freq $baudrate
				$CHIPNAME.swo enable
			} else {
				$CHIPNAME.tpiu configure -protocol uart -output $swv_port -traceclk $trace_clk -pin-freq $baudrate
				$CHIPNAME.tpiu enable
			}
			itm port 0 off
			return "OK"
		}
		stop {
			if {[expr {($USE_SWO == 1)}]} {
				$CHIPNAME.swo disable
			} else {
				$CHIPNAME.tpiu disable
			}
			return "OK"
		}
		info {
			set data [st-link cmd 36 0xf2 0x57 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
			set cmd_result [expr "0x[lindex $data 0]"]
			if {$cmd_result == 0x80} {
				set recommended [extract_uint32 $data 4]
				set n_datasets  [extract_uint32 $data 8]

				set result "OK:[format {%x} $recommended];[format {%x} $n_datasets]"

				for {set i 0} {$i < $n_datasets} {incr i} {
					set index [expr {(3 + ($i * 3)) * 4}]
					set base_freq [extract_uint32_hex $data [expr {$index + 0}]]
					set min_div   [extract_uint32_hex $data [expr {$index + 4}]]
					set max_div   [extract_uint32_hex $data [expr {$index + 8}]]
					set result "$result;$base_freq,$min_div,$max_div"
				}
				set result "$result;"
				return $result
			} elseif {$cmd_result == 0x42} {
				return "OK:7d0;6;7d0,1,1;3e8,1,1;1f4,1,1;fa,1,1;7d,1,1;19,1,1;"
			} else {
				return "Unable to fetch information"
			}
		}
		default {
			return "Unknown argument: $args"
		}
	}
}
