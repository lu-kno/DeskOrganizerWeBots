    
def looper(func):
    '''Wrapper to loop function while continuing simulation until '-1' is returned'''
    def inner(self,*args,**kwargs):
        while self.supervisor.step(self.timestep) != -1:
            self.master.stepOperations()
            if func(self, *args,**kwargs)==-1:
                return
    return inner    

def looperTimeout(func):
    '''Wrapper to loop function while continuing simulation until '-1' is returned or timeout is reached'''
    def inner(self, *args,timeout =10000,**kwargs):
        
        while self.supervisor.step(self.timestep) != -1:
            self.master.stepOperations()
            if func(self, *args,**kwargs)==-1:
                return

            timeout-=self.timestep
            if timeout<0:
                self.logW(f'TIMED OUT: {func.__name__}')
                return
    return inner