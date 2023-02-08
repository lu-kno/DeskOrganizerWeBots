import datetime
from controller import AnsiCodes

class logger():
    def __init__(self,logging='L', logName='Unnamed'):
        self.logging=''.join([i[0].upper() for i in logging.split('_')])
        self.logName=logName
        
    def log(self, *args, **kwargs):
        self.__print('L',*args, color='' , **kwargs)
        
    def logW(self, *args, **kwargs):
        self.__print('W',*args, color=AnsiCodes.YELLOW_FOREGROUND , **kwargs)
        
    def logE(self, *args, **kwargs):
        self.__print('E',*args, color=AnsiCodes.RED_BACKGROUND , **kwargs)
            
    def logD(self, *args, **kwargs):
        if self.logging in ['VV','V','D']:
            self.__print('D',*args, color=AnsiCodes.MAGENTA_FOREGROUND , **kwargs)
            
    def logV(self, *args, **kwargs):
        if self.logging in ['VV','V']:
            self.__print('V',*args, color=AnsiCodes.CYAN_FOREGROUND , **kwargs)
            
    def logVV(self, *args, **kwargs):
        if self.logging in ['VV']:
            self.__print('VV', *args, color=AnsiCodes.GREEN_FOREGROUND , **kwargs)
            
    def __print(self, level, *args, color='', **kwargs):
        statement = ' '.join([i.__str__() for i in args])
        prefix = f'[{datetime.datetime.now().time()}][{self.logName}][{level}]'
        for s in statement.split('\n'):
            print(color,prefix,s,AnsiCodes.RESET,**kwargs)
            prefix = '-'*len(prefix)