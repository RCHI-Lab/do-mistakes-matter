class PID():
    def __init__(self,base,acceptable_range,const_c,const_d):
        self.setpoint=55
        self.base=base
        self.const_c=const_c
        self.const_d=const_d
        self.acceptable_range=acceptable_range
        self.move_by=0
        self.error_prev=0
        self.time_prev=-100

    def update(self, time_new, base_new):
        error_new=self.setpoint-base_new
        move_by=0
        if abs(error_new)> self.acceptable_range:
            move_by = (self.const_c * error_new)+(self.const_d * (error_new-self.error_prev)/(time_new-self.time_prev))
        self.base=base_new+move_by
        self.move_by=move_by #for display
        self.error_prev=error_new
        self.time_prev=time_new
        return (self.base,self.move_by)