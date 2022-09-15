# Helper for counting GDB clients and hooking on last client to detach

proc first_gdb_attach_hook {} {
	# simply do nothing
}

proc last_gdb_detach_hook {} {
	# to close connection if debug mode entered
	shutdown
}

set gdb_clients_num 0
proc gdb_attach_hook {} {
	global gdb_clients_num

	incr gdb_clients_num
	if { $gdb_clients_num == 1 } {
		first_gdb_attach_hook
	}
}

proc gdb_detach_hook {} {
	global gdb_clients_num

	incr gdb_clients_num -1
	if { $gdb_clients_num <= 0 } {
		last_gdb_detach_hook
	}
}
