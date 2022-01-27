set_property trace_limit 1000000000 [current_sim]
#open_wave_config top_tb.wcfg
open_wave_config prefetch_buffer.wcfg
set_property display_limit 1000000000 [current_wave_config]
run 500us;
