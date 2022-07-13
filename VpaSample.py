from http.client import ImproperConnectionState
import time
from Vpa30816 import Vpa30816Componet, Vpa30816ReadCommand


a=Vpa30816Componet(7,1)
time.sleep(1)
a.Start()

a.Set_Read_Bypass(Vpa30816ReadCommand.X_FFT_Frequency18)
while True:
    print(a.Get_Value(Vpa30816ReadCommand.X_FFT_Frequency1))
    print(a.Get_Value(Vpa30816ReadCommand.Y_FFT_Frequency1))
    print(a.Get_Value(Vpa30816ReadCommand.Z_FFT_Frequency1))