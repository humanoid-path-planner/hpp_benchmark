for i in range (ps.numberPaths ()):
    l = ps.pathLength (i)
    t = 0
    while t < l:
        res, msg = robot.isConfigValid (ps.configAtParam (i, t))
        if not res and msg.find ("value out of range") == -1:
            print('at {0}, path {1}: {2}'.format (t,i, msg))
        t += dt
