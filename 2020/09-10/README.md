# Results of benchmarks 2020-09-10

| Benchmark                 | Averarge Time / Average Number of Node | Notes
| ------------------------- |--------------------------------------- |-------------------------------------------------
| baxter-manipulation-boxes |  15.086284699999998/ 319.85            | When collisions or discontinuities are detected we really can see it with  gepetto-gui.
|                           |                                        |
| construction-set          | 6.15189835 / 34.8                      | Nothing to report.
|                           |                                        |
| hrp2-on-the-ground        |  0.21011575  / 21.45                      | The problem with RLEG_LINK0_0 and RLEG_LINK2_0 is fixed. Nothing to report.
|                           |                                        |
| pr2-in-iai-kitchen        | 1.51768605 / 45.85                     | Nothing to report. Collisions and discontinuities are not checked for pr2-in-iai-kitchen
|                           |                                        |
| pr2-manipulation-kitchen  | 17.4098227 / 121.75                    | There is a lot of collisions and the time is long if we compare to others
|                           |                                        |
| pr2-manipulation-two-hand | 5.83269335 / 51.25                       | Nothing to report. When collisions or discontinuities are detected we really can see it with gepetto-gui.
|                           |                                        |
| romeo-placard             |  202.7526188 / 692.0                   | Even with high Threshold (50) most of the paths are discontinuous                                       |
|                           |                                        |
| ur3-spheres               |    18.9818339 / 446.2                  | The update problem is fixed but there are several discontinuities with a threshold=5
