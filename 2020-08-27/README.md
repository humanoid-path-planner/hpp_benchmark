# Results of benchmarks 2020-08-27

| Benchmark                 | Averarge Time / Average Number of Node | Notes                                           
| ------------------------- |--------------------------------------- |-------------------------------------------------
| baxter-manipulation-boxes |   1.4768076 / 32.4                     | When collisions or discontinuities are detected we really can see it with  gepetto-gui.               
|                           |                                        |                                                                                  
| construction-set          | 6.20986195 / 60.95                     | Nothing to report.                              
|                           |                                        |                                                  
| hrp2-on-the-ground        | 1.4768076  / 32.4                      | Collision between object RLEG_LINK0_0 and RLEG_LINK2_0 in most of the path seems like this pair of collision have to be disabled.        
|                           |                                        |                                                  
| pr2-in-iai-kitchen        | 1.48800395 / 45.85                     | Nothing to report. Collisions and discontinuities are not checked for pr2-in-iai-kitchen      
|                           |                                        |                                                 
| pr2-manipulation-kitchen  | 14.27721725 / 96.6                     |  There is a lot of collisions and the time is long if we compare to others                   
| pr2-manipulation-two-hand | 4.6784428 / 44.7                       | Nothing to report. When collisions or discontinuities are detected we really can see it with gepetto-gui.                                                
|                           |                                        |                                        
| romeo-placard             | 101.56022655                           | The time is really high due to the random algorithm. Even with high Threshold (50) most of the paths are discontinuous                                       |                                                 
|                           |                                        |                                                 
| ur3-spheres               |                                        | Nothing to report. The benchmark file is not generated due to a problem of update. "Cannot Import Equality from hpp_idl.hpp"                                                 
