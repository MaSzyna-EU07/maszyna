//$e version: 1
//$e name: Semaphore, 2 arms
//$e category: signals
//$e description: a semaphore of the tests of the descriptions
//$e signal: {kind: semaphore, mount: mast, arms: 2, height: 5.5, aspects: {Sr1: stop, S2: vmax}}
//$e params:
//$e   - {id: 1, role: name, label: Name}
//$e   - {id: 2, role: pos.x}
//$e   - {id: 3, role: free, label: Track, default: tor1}
// the template of the tests of the descriptions (//$e): its content is left as it is
node -1 0 (p1)_mem memcell (p2) 0 0 none 0 0 (p3) endmemcell
