// the tracks of the map of the tests of the migration, included at the origin 1000 0 2000: two alignments meeting at a point, a grade
// line along the second one, one along the first one with a break of no vertical curve, a compound curve the domain has no place for,
// and the lines of the other kinds, which are kept as they are
//$h # horizontal alignments designed in the track editor, the simulation skips these lines
//$h alignment 0.0000 10.0000 0.0000 600.0000 12.0000 600.0000
//$h source designed origin 1200.0000 shape clothoid speed 100.0
//$h ends 0.000000000 1.000000000 1.000000000 0.000000000 0.000000 0.000000 0.000 0.000
//$h norms 1500.0 0.600 150.0 35.0 10.00 0.300 2.00 30.00 0.400
//$h pieces 3 2.000
//$h vertex 0.0000 600.0000 0.0000 500.000 80.000 80.000 60.0 0 0 1.000000000 0
//$h click 0.0000 10.0000 50.0000
//$h end
//$h alignment 600.0000 12.0000 600.0000 1400.0000 14.0000 600.0000
//$h source designed origin 2500.0000 shape parabola speed 120.0
//$h ends 1.000000000 0.000000000 1.000000000 0.000000000 0.000000 0.000000 0.000 0.000
//$h norms 1500.0 0.600 150.0 35.0 10.00 0.300 2.00 30.00 0.400
//$h pieces 1 2.000
//$h end
//$h alignment 2000.0000 0.0000 0.0000 3000.0000 0.0000 2000.0000
//$h source traced origin 0.0000 shape parabola speed 80.0
//$h ends 0.000000000 1.000000000 1.000000000 0.000000000 0.000000 0.000000 1200.000 0.000 turns 1 0 curved 1 0 60.0 0.0
//$h norms 1500.0 0.600 150.0 35.0 10.00 0.300 2.00 30.00 0.400
//$h pieces 2 1.500
//$h kept 10.0000 1500.0000
//$h vertex 2000.0000 2000.0000 0.0000 600.000 50.000 50.000 70.0 0 1 0.500000000 1 900.000 40.000 0.500000000 cants 70.0 50.0 ramps 0.000 -1.000 5.000 60.000
//$h element straight -1 0.0000 1289.6263 0.000 0.000 0.0 2000.0000 0.0000 0.000000000 1.000000000
//$h element transition 0 1289.6263 50.0000 0.000 -600.000 70.0 2000.0000 1289.6263 0.000000000 1.000000000
//$h element arc 0 1339.6263 433.7389 -600.000 -600.000 70.0 2000.6944 1339.6176 0.041654611 0.999132070
//$h element transition 0 1773.3652 40.0000 -600.000 -900.000 70.0 2167.1634 1729.9587 0.692223009 0.721683660
//$h element arc 0 1813.3652 650.6083 -900.000 -900.000 50.0 2195.6917 1757.9894 0.731227954 0.682133183
//$h element transition 0 2463.9735 50.0000 -900.000 0.000 50.0 2784.6148 1999.5371 0.999614222 0.027774206
//$h element straight -1 2513.9735 165.3891 0.000 0.000 0.0 2834.6109 2000.0000 1.000000000 0.000000000
//$h end
//$p # vertical profiles (grade lines) designed in the track editor, the simulation skips these lines
//$p profile 984.3490 600.0000 12.0000 600.0000 0.0000 10.0000 0.0000 0.0000 11.0000 600.0000
//$p speed 100.0 origin 0.0000 join 1 1
//$p norms 6.0000 2.5000 0.4000 2000.0 2.0000 50.00 0.0050
//$p point 0.0000 12.0000 0.0 0
//$p point 500.0000 11.5000 0.0 0
//$p point 984.3490 10.0000 0.0 0
//$p end
//$p profile 800.0000 600.0000 12.0000 600.0000 1400.0000 14.0000 600.0000 1000.0000 13.0000 600.0000
//$p speed 120.0 origin 2500.0000 join 1 1
//$p norms 6.0000 2.5000 0.4000 2000.0 2.0000 50.00 0.0050
//$p point 0.0000 12.0000 0.0 0
//$p point 400.0000 14.0000 10000.0 1
//$p point 800.0000 13.0000 0.0 0
//$p end
//$r # switches placed in the track editor, the simulation skips these lines
//$r switch z1 Rz 60E1 500.000 12.000 L 15.550 15.550 0.000 parts 1 z1 shape 33.2300 0.0000 33.1000 1.8000 start 100.0000 0.0000 200.0000 dir 0.000000000 1.000000000 km 12.345 number 1 source designed
//$r connection c1 Rz 60E1 500.000 12.000 R 15.550 15.550 -1200.000 parts 2 z3 z4 start 110.0000 0.0000 210.0000 dir 0.600000000 0.800000000 km - number - source measured lift 0.0600 110.0000 0.0000 210.0000 150.0000 0.0000 240.0000 ramp 60.000 liftend 0.0400
//$r slip s1 Rkpd 49E1 190.000 9.000 - 16.615 16.615 0.000 parts 4 s1a s1b s1c s1d start 0.0000 0.0000 0.0000 dir 1.000000000 0.000000000 km 0.500 number 5ab source designed middle 300.000
//$k # kilometrage of the lines set in the track editor, the simulation skips these lines
//$k mark 91 100.0000 0.0000 200.0000 way 0.000000000 1.000000000 km 12.345
//$k mark 91 100.0000 0.0000 1200.0000 way 0.000000000 1.000000000 km 13.345 beyond 14.000
//$k via 91 105.0000 0.0000 700.0000
//$t # the tracks whose grade line was fitted in the track editor, with their shape then; the simulation skips these lines
//$t fitted tor_1 00ff00ff12345678
//$t fitted tor_2 0123456789abcdef hand
//$f fitted tor_9 0000000000000001
//$w # the tracks sharing a grade line, each with the number of it; the simulation skips these lines
//$w gradeline tor_1@100,200 3
//$w gradeline tor_2@-50,7 3 hand
//$b # objects bound to the paths in the track editor, they follow changes of the track; the simulation skips these lines
//$b model 2 10.0000 0.5000 20.0000 models/slupy/slup.t3d 1 1 0 0.0000 0.0000 0.0000 0.0000 0.0000 100.0000
//$b memcell 4 12.0000 0.0000 30.0000 sem_mem 0 2 0 0.0000 0.0000 0.0000 0.0000 0.0000 100.0000 1 0.0000 0.0000 0.0000 5.0000 0.0000 100.0000
//$s # stations described in the track editor, the simulation skips these lines
//$s station MMz line 91 km 12.345 from left anchor z1 braking 700 blocks bpd_a=sbl4,bpd_b=semi ways bpd_a=2 joints +100.000/0.000/200.000,-110.500/0.000/205.250 name £Ûdü Kaliska
//$s station - line - km - from right anchor z7 blocks x=sbl name Stacja
//$o on 1 origin 512345.67 645678.90
//$z # the area the editor downloads the orthophoto and the terrain model in, PUWG 1992 easting northing
//$z point 645000.00 512000.00
//$z point 646000.00 512000.00
//$z point 646000.50 513000.25
node -1 0 tor_a track normal 59.4753 1.435 0.15 25 20 0 flat vis rail_screw_used1 4 tpbps-new2 0.2 0.5 1.1 0 10 0 0 0 0 0 0 0 0 10.12 59.4753 0 0 velocity 100 endtrack
