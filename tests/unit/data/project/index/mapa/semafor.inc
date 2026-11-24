// a signal of the map, a template: its memory cell and its events named after the first parameter
node -1 0 (p1)_mem memcell (p2) 0 (p3) none 0 0 (p4) endmemcell
event (p1)_s1 updatevalues 0 (p1)_mem SetVelocity 100 100 endevent
event (p1)_sr2 updatevalues 0.5 (p1)_mem SetVelocity 0 0 endevent
