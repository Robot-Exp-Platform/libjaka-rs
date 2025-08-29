import libjaka.libjaka

def test_func(state: libjaka.ArmState, duration):
    print("state:", state)
    print("duration:", duration)
    motion = libjaka.MotionType.Joint([0, 0, 0, 0, 0, 0])
    return (motion,True)

libjaka.test_closure(test_func)