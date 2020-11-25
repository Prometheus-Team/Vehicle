Class PIDParam():
    def __init__(self, p=0.1, i=0.01,d=0.1):
        self.proportional=p
        self.integral=i
        self.derivative=d
    def setPID(p,i,d):
        self.proportional=p
        self.integral=i
        self.derivative=d