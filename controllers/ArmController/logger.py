import datetime

class logger():
    def __init__(self,logging='L', logName='Unnamed'):
        self.logging=''.join([i[0].upper() for i in logging.split('_')])
        self.logName=logName
        
    def log(self, *args, **kwargs):
        self.__print('L',*args,**kwargs)
        
    def logW(self, *args, **kwargs):
        self.__print('W',*args,**kwargs)
            
    def logD(self, *args, **kwargs):
        if self.logging in ['VV','V','D']:
            self.__print('D',*args,**kwargs)
            
    def logV(self, *args, **kwargs):
        if self.logging in ['VV','V']:
            self.__print('V',*args,**kwargs)
            
    def logVV(self, *args, **kwargs):
        if self.logging in ['VV']:
            self.__print('VV', *args,**kwargs)
            
    def __print(self, level, *args, **kwargs):
        print(f'[{datetime.datetime.now().time()}][{self.logName}][{level}]',*args,**kwargs)