
from __future__ import annotations
import datetime
from controller import AnsiCodes
import typing
from typing import Optional, Iterable, Literal, Any, NewType


class logger():
    def __init__(self, logging: str = 'L', logName: str = 'Unnamed'):
        self.logging = logging.upper()[0]
        if self.logging=='V' and 'V' in logging.upper()[1:]:
            self.logging='VV'

        self.logName=logName
        
    def log(self, *args, **kwargs) -> None:
        self.__print('L',*args, color='' , **kwargs)
        
    def logW(self, *args, **kwargs) -> None:
        self.__print('W',*args, color=AnsiCodes.YELLOW_FOREGROUND , **kwargs)
        
    def logE(self, *args, **kwargs) -> None:
        self.__print('E',*args, color=AnsiCodes.RED_BACKGROUND , **kwargs)
            
    def logD(self, *args, **kwargs) -> None:
        if self.logging in ['VV','V','D']:
            self.__print('D',*args, color=AnsiCodes.MAGENTA_FOREGROUND , **kwargs)
            
    def logV(self, *args, **kwargs) -> None:
        if self.logging in ['VV','V']:
            self.__print('V',*args, color=AnsiCodes.CYAN_FOREGROUND , **kwargs)
            
    def logVV(self, *args, **kwargs) -> None:
        if self.logging in ['VV']:
            self.__print('VV', *args, color=AnsiCodes.GREEN_FOREGROUND , **kwargs)
            
    def __print(self, level, *args, color: int|str|None = '', **kwargs) -> None:
        statement = ' '.join([i.__str__() for i in args])
        prefix = f'[{datetime.datetime.now().time()}][{self.logName}][{level}]'
        for s in statement.split('\n'):
            print(color,prefix,s,AnsiCodes.RESET,**kwargs)
            prefix = '-'*len(prefix)
            
            
    
def looper(func: typing.Callable) -> typing.Callable:
    '''Wrapper to loop function while continuing simulation until '-1' is returned'''
    def inner(self,*args,**kwargs):
        while self.supervisor.step(self.timestep) != -1:
            self.master.stepOperations()
            if func(self, *args,**kwargs)==-1:
                return
    return inner    

def looperTimeout(func: typing.Callable) -> typing.Callable:
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