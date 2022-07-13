from enum import IntEnum
from math import fabs
from operator import itemgetter
from sys import hexversion
import time
from traceback import print_tb
from unittest.result import failfast
from django.forms import SlugField
from numpy import byte, true_divide
from pymysql import DatabaseError
import serial
import struct
import threading
class SerialPort(threading.Thread):
    def __init__(self,analyzeFunction,comport="/dev/ttyUSB0",baudRate=9600,byteSize=serial.EIGHTBITS,stopBit=serial.STOPBITS_ONE,Parity=serial.PARITY_NONE):
        self.socketBind =threading.Thread(target= self.ReadThread)
        self.serial_port = serial.Serial(
            port=comport,
            baudrate=baudRate,
            bytesize=byteSize,
            parity=Parity,
            stopbits=stopBit)
        self.asdasd=0
        self.__analyzeFunction=analyzeFunction
        self.cycleRunFlag=True
        self.socketBind.start()
        self.serial_port.baudrate
    def SendCommand(self,command):
        self.serial_port.write(command)
    def SystemClose(self):
        self.cycleRunFlag=False
    def ReadThread(self):
        while self.cycleRunFlag:
            if self.serial_port.inWaiting() > 0:
                data = self.serial_port.read(1)
                self.__analyzeFunction(data[0])

class ModbusControler:
    def __init__(self,comport='COM6',baud=115200) :
        self.SerialComponet=SerialPort(self.ReciveData,comport,baud)
        self.childList=[]
        self.DataBuffer=[]
        self.lock=threading.Lock()
        self.Comport=comport
        pass
    
    def ReciveData(self,data):
        self.DataBuffer.append(data)
        for item in self.childList:
            if item._ReadData(self.DataBuffer):
                return None
        pass
    def SendCommand(self,command):
        self.lock.acquire()
        self.SerialComponet.SendCommand(command)
        self.lock.release()
    def GetComport(self):
        return self.Comport
    def Remove_DataBuffer(self,lenght):
        del self.DataBuffer[0:lenght]
    def Change_Baud(self,baudrate=int):
        self.SerialComponet.serial_port.baudrate=baudrate
class Vpa30816ReadSize(IntEnum):
    Float=2
    Uint32=2
    Uint16=1
    Bool=1
    Byte2=1
    Byte4=2

class Vpa30816ReadCommand(IntEnum):
    X_Mean=0x00
    X_Standard_Deviation=0x02
    X_RMS=0x04
    X_Creat_Factor=0x06
    X_Skewness=0x08
    X_Kurtosis=0x0a
    X_Maximum=0x0c
    X_Minimum=0x0e
    X_Peak_To_Peak=0x10
    X_Speed=0x12
    Y_Mean=0x14
    Y_Standard_Deviation=0x16
    Y_RMS=0x18
    Y_Creat_Factor=0x1a
    Y_Skewness=0x1c
    Y_Kurtosis=0x1e
    Y_Maximum=0x20
    Y_Minimum=0x22
    Y_Peak_To_Peak=0x24
    Y_Speed=0x26
    Z_Mean=0x28
    Z_Standard_Deviation=0x2a
    Z_RMS=0x2c
    Z_Creat_Factor=0x2e
    Z_Skewness=0x30
    Z_Kurtosis=0x32
    Z_Maximum=0x34
    Z_Minimum=0x36
    Z_Peak_To_Peak=0x38
    Z_Speed=0x3a
    Temperature=0x3c

    X_FFT_Frequency1=256
    X_FFT_Strength1=258
    X_FFT_Frequency2=260
    X_FFT_Strength2=262
    X_FFT_Frequency3=264
    X_FFT_Strength3=266
    X_FFT_Frequency4=268
    X_FFT_Strength4=270
    X_FFT_Frequency5=272
    X_FFT_Strength5=274
    X_FFT_Frequency6=276
    X_FFT_Strength6=278
    X_FFT_Frequency7=280
    X_FFT_Strength7=282
    X_FFT_Frequency8=284
    X_FFT_Strength8=286
    X_FFT_Frequency9=288
    X_FFT_Strength9=290
    X_FFT_Frequency10=292
    X_FFT_Strength10=294
    
    X_FFT_Frequency11=296
    X_FFT_Strength11=298
    X_FFT_Frequency12=300
    X_FFT_Strength12=302
    X_FFT_Frequency13=304
    X_FFT_Strength13=306
    X_FFT_Frequency14=308
    X_FFT_Strength14=310
    X_FFT_Frequency15=312
    X_FFT_Strength15=314
    X_FFT_Frequency16=316
    X_FFT_Strength16=318
    X_FFT_Frequency17=320
    X_FFT_Strength17=322
    X_FFT_Frequency18=324
    X_FFT_Strength18=326
    X_FFT_Frequency19=328
    X_FFT_Strength19=330
    X_FFT_Frequency20=332
    X_FFT_Strength20=334

    X_FFT_Frequency21=336
    X_FFT_Strength21=338
    X_FFT_Frequency22=340
    X_FFT_Strength22=342
    X_FFT_Frequency23=344
    X_FFT_Strength23=346
    X_FFT_Frequency24=348
    X_FFT_Strength24=350
    X_FFT_Frequency25=352
    X_FFT_Strength25=354
    X_FFT_Frequency26=356
    X_FFT_Strength26=358
    X_FFT_Frequency27=360
    X_FFT_Strength27=362
    X_FFT_Frequency28=364
    X_FFT_Strength28=366
    X_FFT_Frequency29=368
    X_FFT_Strength29=370
    X_FFT_Frequency30=372
    X_FFT_Strength30=374

    X_FFT_Frequency31=376
    X_FFT_Strength31=378
    X_FFT_Frequency32=380
    X_FFT_Strength32=382
    X_FFT_Frequency33=384
    X_FFT_Strength33=386
    X_FFT_Frequency34=388
    X_FFT_Strength34=390
    X_FFT_Frequency35=392
    X_FFT_Strength35=394
    X_FFT_Frequency36=396
    X_FFT_Strength36=398
    X_FFT_Frequency37=400
    X_FFT_Strength37=402
    X_FFT_Frequency38=404
    X_FFT_Strength38=406
    X_FFT_Frequency39=408
    X_FFT_Strength39=410
    X_FFT_Frequency40=412
    X_FFT_Strength40=414

    X_FFT_Frequency41=416
    X_FFT_Strength41=418
    X_FFT_Frequency42=420
    X_FFT_Strength42=422
    X_FFT_Frequency43=424
    X_FFT_Strength43=426
    X_FFT_Frequency44=428
    X_FFT_Strength44=430
    X_FFT_Frequency45=432
    X_FFT_Strength45=434
    X_FFT_Frequency46=436
    X_FFT_Strength46=438
    X_FFT_Frequency47=440
    X_FFT_Strength47=442
    X_FFT_Frequency48=444
    X_FFT_Strength48=446
    X_FFT_Frequency49=448
    X_FFT_Strength49=450
    X_FFT_Frequency50=452
    X_FFT_Strength50=454
    
    Y_FFT_Frequency1=512
    Y_FFT_Strength1=514
    Y_FFT_Frequency2=516
    Y_FFT_Strength2=518
    Y_FFT_Frequency3=520
    Y_FFT_Strength3=522
    Y_FFT_Frequency4=524
    Y_FFT_Strength4=526
    Y_FFT_Frequency5=528
    Y_FFT_Strength5=530
    Y_FFT_Frequency6=532
    Y_FFT_Strength6=534
    Y_FFT_Frequency7=536
    Y_FFT_Strength7=538
    Y_FFT_Frequency8=540
    Y_FFT_Strength8=542
    Y_FFT_Frequency9=544
    Y_FFT_Strength9=546
    Y_FFT_Frequency10=548
    Y_FFT_Strength10=550
    
    Y_FFT_Frequency11=552
    Y_FFT_Strength11=554
    Y_FFT_Frequency12=556
    Y_FFT_Strength12=558
    Y_FFT_Frequency13=560
    Y_FFT_Strength13=562
    Y_FFT_Frequency14=564
    Y_FFT_Strength14=566
    Y_FFT_Frequency15=568
    Y_FFT_Strength15=570
    Y_FFT_Frequency16=572
    Y_FFT_Strength16=574
    Y_FFT_Frequency17=576
    Y_FFT_Strength17=578
    Y_FFT_Frequency18=580
    Y_FFT_Strength18=582
    Y_FFT_Frequency19=584
    Y_FFT_Strength19=586
    Y_FFT_Frequency20=588
    Y_FFT_Strength20=590

    Y_FFT_Frequency21=592
    Y_FFT_Strength21=594
    Y_FFT_Frequency22=596
    Y_FFT_Strength22=598
    Y_FFT_Frequency23=600
    Y_FFT_Strength23=602
    Y_FFT_Frequency24=604
    Y_FFT_Strength24=606
    Y_FFT_Frequency25=608
    Y_FFT_Strength25=610
    Y_FFT_Frequency26=612
    Y_FFT_Strength26=614
    Y_FFT_Frequency27=616
    Y_FFT_Strength27=618
    Y_FFT_Frequency28=620
    Y_FFT_Strength28=622
    Y_FFT_Frequency29=624
    Y_FFT_Strength29=626
    Y_FFT_Frequency30=628
    Y_FFT_Strength30=630

    Y_FFT_Frequency31=632
    Y_FFT_Strength31=634
    Y_FFT_Frequency32=636
    Y_FFT_Strength32=638
    Y_FFT_Frequency33=640
    Y_FFT_Strength33=642
    Y_FFT_Frequency34=644
    Y_FFT_Strength34=646
    Y_FFT_Frequency35=648
    Y_FFT_Strength35=650
    Y_FFT_Frequency36=652
    Y_FFT_Strength36=654
    Y_FFT_Frequency37=656
    Y_FFT_Strength37=658
    Y_FFT_Frequency38=660
    Y_FFT_Strength38=662
    Y_FFT_Frequency39=664
    Y_FFT_Strength39=666
    Y_FFT_Frequency40=668
    Y_FFT_Strength40=670

    Y_FFT_Frequency41=672
    Y_FFT_Strength41=674
    Y_FFT_Frequency42=676
    Y_FFT_Strength42=678
    Y_FFT_Frequency43=680
    Y_FFT_Strength43=682
    Y_FFT_Frequency44=684
    Y_FFT_Strength44=686
    Y_FFT_Frequency45=688
    Y_FFT_Strength45=690
    Y_FFT_Frequency46=692
    Y_FFT_Strength46=694
    Y_FFT_Frequency47=696
    Y_FFT_Strength47=698
    Y_FFT_Frequency48=700
    Y_FFT_Strength48=702
    Y_FFT_Frequency49=704
    Y_FFT_Strength49=706
    Y_FFT_Frequency50=708
    Y_FFT_Strength50=710
    
    Z_FFT_Frequency1=768
    Z_FFT_Strength1=770
    Z_FFT_Frequency2=772
    Z_FFT_Strength2=774
    Z_FFT_Frequency3=776
    Z_FFT_Strength3=778
    Z_FFT_Frequency4=780
    Z_FFT_Strength4=782
    Z_FFT_Frequency5=784
    Z_FFT_Strength5=786
    Z_FFT_Frequency6=788
    Z_FFT_Strength6=790
    Z_FFT_Frequency7=792
    Z_FFT_Strength7=794
    Z_FFT_Frequency8=796
    Z_FFT_Strength8=798
    Z_FFT_Frequency9=800
    Z_FFT_Strength9=802
    Z_FFT_Frequency10=804
    Z_FFT_Strength10=806
    
    Z_FFT_Frequency11=808
    Z_FFT_Strength11=810
    Z_FFT_Frequency12=812
    Z_FFT_Strength12=814
    Z_FFT_Frequency13=816
    Z_FFT_Strength13=818
    Z_FFT_Frequency14=820
    Z_FFT_Strength14=822
    Z_FFT_Frequency15=824
    Z_FFT_Strength15=826
    Z_FFT_Frequency16=828
    Z_FFT_Strength16=830
    Z_FFT_Frequency17=832
    Z_FFT_Strength17=834
    Z_FFT_Frequency18=836
    Z_FFT_Strength18=838
    Z_FFT_Frequency19=840
    Z_FFT_Strength19=842
    Z_FFT_Frequency20=844
    Z_FFT_Strength20=846

    Z_FFT_Frequency21=848
    Z_FFT_Strength21=850
    Z_FFT_Frequency22=852
    Z_FFT_Strength22=854
    Z_FFT_Frequency23=856
    Z_FFT_Strength23=858
    Z_FFT_Frequency24=860
    Z_FFT_Strength24=862
    Z_FFT_Frequency25=864
    Z_FFT_Strength25=866
    Z_FFT_Frequency26=868
    Z_FFT_Strength26=870
    Z_FFT_Frequency27=872
    Z_FFT_Strength27=874
    Z_FFT_Frequency28=8476
    Z_FFT_Strength28=878
    Z_FFT_Frequency29=880
    Z_FFT_Strength29=882
    Z_FFT_Frequency30=884
    Z_FFT_Strength30=886

    Z_FFT_Frequency31=888
    Z_FFT_Strength31=890
    Z_FFT_Frequency32=892
    Z_FFT_Strength32=894
    Z_FFT_Frequency33=896
    Z_FFT_Strength33=898
    Z_FFT_Frequency34=900
    Z_FFT_Strength34=902
    Z_FFT_Frequency35=904
    Z_FFT_Strength35=906
    Z_FFT_Frequency36=908
    Z_FFT_Strength36=910
    Z_FFT_Frequency37=912
    Z_FFT_Strength37=914
    Z_FFT_Frequency38=916
    Z_FFT_Strength38=918
    Z_FFT_Frequency39=920
    Z_FFT_Strength39=922
    Z_FFT_Frequency40=924
    Z_FFT_Strength40=926

    Z_FFT_Frequency41=928
    Z_FFT_Strength41=930
    Z_FFT_Frequency42=932
    Z_FFT_Strength42=934
    Z_FFT_Frequency43=936
    Z_FFT_Strength43=938
    Z_FFT_Frequency44=940
    Z_FFT_Strength44=942
    Z_FFT_Frequency45=944
    Z_FFT_Strength45=946
    Z_FFT_Frequency46=948
    Z_FFT_Strength46=950
    Z_FFT_Frequency47=952
    Z_FFT_Strength47=954
    Z_FFT_Frequency48=956
    Z_FFT_Strength48=958
    Z_FFT_Frequency49=960
    Z_FFT_Strength49=962
    Z_FFT_Frequency50=964
    Z_FFT_Strength50=966

    Version=0x400
    ODR=0x402
    G_Range=0x403
    Baud=0x404
    Modbus_Address=0x405
    X_Axis_Bias=0x40c
    Y_Axis_Bias=0x40e
    Z_Axis_Bias=0x410
    Temperature_Bias=0x410
    SN12=0x414
    SN22=0x416
    RPM=0x418
    Hi_Pass_Filter=0x419
    Model_Name12=0x700
    # Model_Name22=0x701
    UID16=0x710
    UID26=0x711
    UID36=0x712
    UID46=0x713
    UID56=0x714
    # UID66=0x715

class Vpa30816WriteCommand(IntEnum):
    Reset=0x00
    Stop=0x01
    Restart=0x02
    ODR=0x03
    G_Range=0x04
    RS485_Baud=0x05
    Reserved=0x06
    Device_Modbus_Address=0x07
    X_Axis_Bias=0x08
    Y_Axis_Bias=0x09
    Z_Axis_Bias=0x0a
    Temperature_Bisa=0x0b
    Reset_Bias=0x0c
    Hi_pass_filter=0x0d
    RPM=0x0e
    Auto_Zeroing=0x20
    SN14=0x30
    SN24=0x31
    SN34=0x32
    SN44=0x33
    Unlock_Write_SN=0xff
class ODR_Parameter(IntEnum):
    ODR_26667Hz=0
    ODR_13333Hz=1
    ODR_6667Hz=2
    ODR_3333Hz=3
    ODR_1667Hz=4
    ODR_833Hz=5
class G_Range_Parameter(IntEnum):
    Range_2G=0
    Range_4G=1
    Range_8G=2
    Range_16G=3
class Baud_Parameter(IntEnum):
    Baud_9600=0
    Baud_19200=1
    Baud_57600=2
    Baud_115200=3
    Baud_230400=4
    Baud_256000=5
    Baud_460800=6
    Baud_921600=7
    Baud_1MB=8
    Baud_2MB=9
    Baud_4MB=10
class Rest_Bisa_Parameter(IntEnum):
    X=0
    Y=1
    Z=2
    Temperature=3


class Vpa30816Componet():
    ModbusControlers= None

    def __init__(self,comport=int,station=int,baud=115200) :
        try :
            self.Station=station
            self.Comport="COM"+str(comport)
            Vpa30816Componet.CreateControler(self.Comport)
            self._ModbusControler=Vpa30816Componet.GetModbusControler(self.Comport)
            self._ModbusControler.childList.append(self)
            self._cycleRun =threading.Thread(target= self.threadCycleRun)
            self._cycleRunFlag=True 
            self._startRun=False 
            self._fristRun=True 
            self._readLenght=3
            self._readCommand=Vpa30816ReadCommand.X_Mean
            self._writeCommand=Vpa30816WriteCommand.Auto_Zeroing
            self._commandFinish=True
            self._readByPassList=[]
            self.startRunTime=time.time()
            self.ReadValue={Vpa30816ReadCommand:float}
            self._ReadCommandOrderList=[]
            self._commandSendTime=time.time()
            for item in Vpa30816ReadCommand:
                self.ReadValue[item]=0
                self._ReadCommandOrderList.append(item)
            self.ReadValue[Vpa30816ReadCommand.Modbus_Address]=0
            self.ODR=ODR_Parameter.ODR_13333Hz
            self.X_Axis_Bias=0
            self.Y_Axis_Bias=0
            self.Z_Axis_Bias=0
            self.Temperature_Bisa=0
            self.Hi_passFilter=False
            self.RPM=0
            self.Baudrate=baud
            self._cycleRun.start()
            
        except serial.SerialException as ex:
            print('Comport Error: ',ex)
            self.Station=-1
            pass
    def Start(self)->bool:
        if self._WiterCommand( Vpa30816WriteCommand.Restart):
            time.sleep(1)
            self.startRunTime=time.time()
            self._startRun=True 
            return True
    def Stop(self)->bool:
        if self._WiterCommand( Vpa30816WriteCommand.Stop):
            self._startRun=False 
            return True 
    def ReStart(self)->bool:
        if self._WiterCommand( Vpa30816WriteCommand.Restart):
            self._startRun=True
            return True 
        pass
    def Get_Value(self,data=Vpa30816ReadCommand):
        return self.ReadValue[data]
    def Change_ODR(self,parameter=ODR_Parameter)->bool:
        return self._WiterCommand( Vpa30816WriteCommand.ODR,parameter)
    def Change_G_Raange(self,parameter=G_Range_Parameter)->bool:
        return self._WiterCommand( Vpa30816WriteCommand.G_Range,parameter)
    def Change_Baud(self,parameter=Baud_Parameter)->bool:
        return self._WiterCommand( Vpa30816WriteCommand.RS485_Baud,parameter)
    def Change_Modbus_Address(self,parameter=int)->bool:
        return self._WiterCommand( Vpa30816WriteCommand.Device_Modbus_Address,parameter)
    def Change_X_Axis_BIAS(self,parameter=int)->bool:
        return self._WiterCommand( Vpa30816WriteCommand.X_Axis_Bias,parameter)
    def Change_Y_Axis_BIAS(self,parameter=int)->bool:
        return self._WiterCommand( Vpa30816WriteCommand.Y_Axis_Bias,parameter)
    def Change_Z_Axis_BIAS(self,parameter=int)->bool:
        return self._WiterCommand( Vpa30816WriteCommand.Z_Axis_Bias,parameter)
    def Change_Temperature_Axis_BIAS(self,parameter=int)->bool:
        return self._WiterCommand( Vpa30816WriteCommand.Temperature_Bisa,parameter)
    def Change_Reset_Bias(self,parameter=Rest_Bisa_Parameter)->bool:
        return self._WiterCommand( Vpa30816WriteCommand.Reset_Bias,parameter)
    def Change_Hi_pass_filter(self,parameter=bool)->bool:
        param=0
        if parameter :
            param =1
        return self._WiterCommand( Vpa30816WriteCommand.Hi_pass_filter,param)
    def Change_RPM(self,parameter=int)->bool:
        return self._WiterCommand( Vpa30816WriteCommand.RPM,parameter)
    def Change_Auto_Zeroing(self)->bool:
        return self._WiterCommand( Vpa30816WriteCommand.Auto_Zeroing)
    def Set_Read_Bypass(self,readCommand=Vpa30816ReadCommand):
        if (readCommand in self._readByPassList)==False:
            self._readByPassList.append(readCommand)
    def Cancel_Read_Bypass(self,readCommand=Vpa30816ReadCommand):
        if readCommand in self._readByPassList:
            self._readByPassList.remove(readCommand)
    def threadCycleRun(self):
        index = self._ReadCommandOrderList.index(Vpa30816ReadCommand.Version)
        time.sleep(2)
        while(self._cycleRunFlag):
            if self._startRun:
                if self._checkCommadFinish():
                    item=self._ReadCommandOrderList[index]
                    index+=1
                    if (item in self._readByPassList) ==False:
                        if item < Vpa30816ReadCommand.Version:
                            self.ReadCommand(item)
                    if index>=len(self._ReadCommandOrderList) or self._ReadCommandOrderList[index]>= Vpa30816ReadCommand.Version:
                        index=0
                        time.sleep(0.002)
            elif self._fristRun:
                if index<len(self._ReadCommandOrderList)-1:
                    if self._commandFinish:
                        index+=1
                        self.ReadCommand( self._ReadCommandOrderList[index])
                else:
                    self._fristRun=False
                    index=0
            time.sleep(0.002)
            

    def Chagne_Station(self,station=int)->bool:
        for item  in self._ModbusControler.childList:
            if item.Station == station:
                return False
        return self._WiterCommand( Vpa30816WriteCommand.Device_Modbus_Address,station)
        
    def _checkCommadFinish(self)->bool:
        if self._commandFinish:
            return True
        elif  (time.time()-self._commandSendTime)>2:
            return True
        return False
    def _ReadData(self,dataBuffer)->bool:
        if dataBuffer[0]==self.Station and len(dataBuffer)>=self._readLenght:
            if dataBuffer[1]==3:#Read
                if self._readLenght==3:
                    self._readLenght=dataBuffer[2]+5
                else:
                    value=0
                    strValue=""
                    readValueType=self._ReadDataType(self._readCommand)
                    if readValueType == Vpa30816ReadSize.Float:
                        for i in range(0,dataBuffer[2]):
                            strValue=strValue+hex(dataBuffer[-3-i]).replace('0x','').zfill(2)
                        value=self.HexToFloat(strValue)
                    elif self._readCommand == Vpa30816ReadCommand.Version:
                        value=str(dataBuffer[3]).zfill(2)+'.'+str(dataBuffer[4]).zfill(2)+'.'+str(dataBuffer[5]).zfill(2)+'+'+str(dataBuffer[6]).zfill(2)
                    elif readValueType== Vpa30816ReadSize.Uint16:
                        value=int(hex(dataBuffer[3]).replace('0x','')+hex(dataBuffer[4]).replace('0x','').zfill(2),16)
                    else:
                        for i in range(0,dataBuffer[2]):
                            value+=hex(dataBuffer[-3-i])
                    self.ReadValue[self._readCommand]=value
                    self._ModbusControler.Remove_DataBuffer(self._readLenght)
                    self._readLenght=3
                    self._commandFinish=True
                    if(self._startRun==False):
                        self.Station=self.ReadValue[Vpa30816ReadCommand.Modbus_Address]
                        self.ODR=ODR_Parameter(self.ReadValue[Vpa30816ReadCommand.ODR])
                        self.Baudrate=Baud_Parameter(self.ReadValue[Vpa30816ReadCommand.Baud])
                        self.X_Axis_Bias=self.ReadValue[Vpa30816ReadCommand.X_Axis_Bias]
                        self.Y_Axis_Bias=self.ReadValue[Vpa30816ReadCommand.Y_Axis_Bias]
                        self.Z_Axis_Bias=self.ReadValue[Vpa30816ReadCommand.Z_Axis_Bias]
                        self.Temperature_Bisa=self.ReadValue[Vpa30816ReadCommand.Temperature_Bias]
                        self.Hi_passFilter=self.ReadValue[Vpa30816ReadCommand.Hi_Pass_Filter]
                        self.RPM=self.ReadValue[Vpa30816ReadCommand.RPM]

                    return True
                pass
            elif len(dataBuffer)==8 :#Write
                if self._writeCommand == Vpa30816WriteCommand.Device_Modbus_Address:
                    self.Station=dataBuffer[5]
                elif self._writeCommand == Vpa30816WriteCommand.ODR:
                    self.ODR=ODR_Parameter(dataBuffer[5])
                elif self._writeCommand == Vpa30816WriteCommand.RS485_Baud:
                    if Baud_Parameter(dataBuffer[5])== Baud_Parameter.Baud_9600:
                        self._ModbusControler.Change_Baud(9600)
                        self.Baudrate=9600
                    elif Baud_Parameter(dataBuffer[5])== Baud_Parameter.Baud_19200:
                        self._ModbusControler.Change_Baud(19200)
                        self.Baudrate=19200
                    elif Baud_Parameter(dataBuffer[5])== Baud_Parameter.Baud_57600:
                        self._ModbusControler.Change_Baud(57600)
                        self.Baudrate=57600
                    elif Baud_Parameter(dataBuffer[5])== Baud_Parameter.Baud_115200:
                        self._ModbusControler.Change_Baud(115200)
                        self.Baudrate=115200
                    elif Baud_Parameter(dataBuffer[5])== Baud_Parameter.Baud_230400:
                        self._ModbusControler.Change_Baud(230400)
                        self.Baudrate=230400
                    elif Baud_Parameter(dataBuffer[5])== Baud_Parameter.Baud_256000:
                        self._ModbusControler.Change_Baud(256000)
                        self.Baudrate=256000
                    elif Baud_Parameter(dataBuffer[5])== Baud_Parameter.Baud_460800:
                        self._ModbusControler.Change_Baud(460800)
                        self.Baudrate=460800
                    elif Baud_Parameter(dataBuffer[5])== Baud_Parameter.Baud_921600:
                        self._ModbusControler.Change_Baud(921600)
                        self.Baudrate=921600
                    elif Baud_Parameter(dataBuffer[5])== Baud_Parameter.Baud_1MB:
                        self._ModbusControler.Change_Baud(1024000)
                        self.Baudrate=1024000
                    elif Baud_Parameter(dataBuffer[5])== Baud_Parameter.Baud_2MB:
                        self._ModbusControler.Change_Baud(2048000)
                        self.Baudrate=2048000
                elif self._writeCommand == Vpa30816WriteCommand.X_Axis_Bias:
                    self.X_Axis_Bias=int(hex(dataBuffer[4]).replace('0x','')+hex(dataBuffer[5]).replace('0x','').zfill(2),16)
                elif self._writeCommand == Vpa30816WriteCommand.Y_Axis_Bias:
                    self.Y_Axis_Bias=int(hex(dataBuffer[4]).replace('0x','')+hex(dataBuffer[5]).replace('0x','').zfill(2),16)
                elif self._writeCommand == Vpa30816WriteCommand.Z_Axis_Bias:
                    self.Z_Axis_Bias=int(hex(dataBuffer[4]).replace('0x','')+hex(dataBuffer[5]).replace('0x','').zfill(2),16)
                elif self._writeCommand == Vpa30816WriteCommand.Temperature_Bisa:
                    self.Temperature_Bisa=int(hex(dataBuffer[4]).replace('0x','')+hex(dataBuffer[5]).replace('0x','').zfill(2),16)
                elif self._writeCommand == Vpa30816WriteCommand.Hi_pass_filter:
                    self.Hi_passFilter=dataBuffer[5]==1
                elif self._writeCommand == Vpa30816WriteCommand.RPM:
                    self.RPM=int(hex(dataBuffer[4]).replace('0x','')+hex(dataBuffer[5]).replace('0x','').zfill(2),16)
                self._ModbusControler.Remove_DataBuffer(8)
                self._commandFinish=True
        return False

    def _WiterCommand(self,command= Vpa30816WriteCommand.Stop ,parameter=0)->bool:
        if self.Station<0 or self._checkCommadFinish()==False:
            return False
        commandData = bytearray()
        commandData.append(self.Station)
        commandData.append(6)
        cmd =hex(command).replace('0x','').zfill(4)
        commandData.append(int( cmd[0:2],16))
        commandData.append(int( cmd[2:],16))
        param=hex(parameter&0xffff).replace('0x','').zfill(4)
        commandData.append(int(param[0:2],16))
        commandData.append(int(param[2::],16))
        crcData=self._crc(commandData)
        crcData=str(crcData).replace('0x','').zfill(4)
        commandData.append(int(crcData[2::],16))
        commandData.append(int(crcData[0:2],16))
        self._writeCommand=command
        self._commandFinish=False
        self._commandSendTime=time.time()
        self._ModbusControler.SendCommand(commandData)
        return True
    def _ReadDataType(self,command=Vpa30816ReadCommand):
        cmd=int(command)
        if cmd<0x400 or (cmd>=0x40c and cmd <=0x412):
            readLen=Vpa30816ReadSize.Float
        elif cmd == Vpa30816ReadCommand.Version:
            readLen=Vpa30816ReadSize.Uint32
        elif (cmd >=0x402 and cmd <=0x405) or cmd ==Vpa30816ReadCommand.RPM:
            readLen=Vpa30816ReadSize.Uint16
        elif cmd == Vpa30816ReadCommand.Hi_Pass_Filter:
            readLen=Vpa30816ReadSize.Bool
        elif cmd == Vpa30816ReadCommand.Model_Name12:
            readLen=Vpa30816ReadSize.Byte2
        else:
            readLen=Vpa30816ReadSize.Byte4
        return readLen
    def ReadCommand(self,command= Vpa30816ReadCommand.X_Mean )->bool:
        if self.Station<0 and self._checkCommadFinish()==False:
            return False
        commandData = bytearray()
        commandData.append(self.Station)
        commandData.append(0x03)
        readLen=0
        readLen=self._ReadDataType(command)
        cmd =hex(command).replace('0x','').zfill(4)
        commandData.append(int( cmd[0:2],16))
        commandData.append(int( cmd[2:],16))
        commandData.append(0x00)
        commandData.append(readLen)
        crcData=self._crc(commandData)
        crcData=str(crcData).replace('0x','').zfill(4)
        commandData.append(int(crcData[2::],16))
        commandData.append(int(crcData[0:2],16))
        self._readCommand=command
        self._commandFinish=False
        self._commandSendTime=time.time()
        self._ModbusControler.SendCommand(commandData)
        return True
    def _crc(self,value=bytearray):
        xorInt=int('a001',16)
        initInt=int('ffff',16)
        for j in value:
            Value = j
            Value ^=initInt
            for i in range(8):
                xorSwitch=Value&1
                Value=Value>>1
                if xorSwitch ==1:
                    Value ^=xorInt
                initInt=Value
        return hex(Value)
    def FloatToHex(self,floatValue):
        return struct.pack('>f',floatValue).hex()
    def HexToFloat(self,HexValue):
        return float(struct.unpack('<f',bytes.fromhex(HexValue)) [0])
    @classmethod
    def CreateControler(clz,comport):
        is_live=False
        if clz.ModbusControlers is None:
            clz.ModbusControlers=[]
        else:
            for item in clz.ModbusControlers:
                if item.GetComport() == comport:
                    is_live=True
                    break
        if is_live==False:
            ModbusControl=ModbusControler(comport)  
            clz.ModbusControlers.append(ModbusControl)
    @classmethod
    def GetModbusControler(clz,comport)-> ModbusControler:
        for item in clz.ModbusControlers:
            if item.GetComport() == comport:
                return item

