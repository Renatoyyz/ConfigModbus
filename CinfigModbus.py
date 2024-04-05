import serial
from time import sleep
import threading
import math
from typing import Any
import RPi.GPIO as GPIO

class RTU485_Config(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)

        self.ser = serial.Serial(
                                port='/dev/ttyUSB0',  # Porta serial padrão no Raspberry Pi 4
                                # port='/dev/ttyS0',  # Porta serial padrão no Raspberry Pi 4
                                baudrate=9600,       # Taxa de baud
                                bytesize=8,
                                parity="N",
                                stopbits=1,
                                timeout=1,            # Timeout de leitura
                                #xonxoff=False,         # Controle de fluxo por software (XON/XOFF)
                                #rtscts=True
                            )
        
    def close(self):
        self.ser.close()

    def crc16_modbus(self, data):
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if (crc & 0x0001):
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc
    
    def broadcast_config_address(self, adr=1):
        
        dados_recebidos = None
        adr_loc = adr

        id_loc = "00" # Endereço de broadcast
        id_loc = id_loc.zfill(2).upper()

        adr_loc = str(adr_loc)
        adr_loc = adr_loc.zfill(4).upper()

        hex_text = f"{id_loc}060064{adr_loc}"

        bytes_hex = bytes.fromhex(hex_text) # Transforma em hexa

        crc_result = self.crc16_modbus(bytes_hex) # Retorna o CRC

        parte_superior = (crc_result >> 8) & 0xFF  # Desloca 8 bits para a direita e aplica a máscara 0xFF
        parte_inferior = crc_result & 0xFF        # Aplica a máscara 0xFF diretamente

        id_loc = 0 # self._devices_address['mod-adam']

        # Repete-se os comandos em decimal com os devidos bytes de CRC
        self.ser.write([id_loc, 6,0,0x64,0,adr,parte_inferior,parte_superior])

        while self.ser.readable()==False:
            pass
        dados_recebidos = self.ser.read(8)
        try:
            dados_recebidos = dados_recebidos.hex()
            hex_text = dados_recebidos[0:2]+dados_recebidos[2:4]+dados_recebidos[4:6]+dados_recebidos[6:8]+dados_recebidos[8:10]+dados_recebidos[10:12]
            bytes_hex = bytes.fromhex(hex_text) # Transforma em hexa
            crc_result = self.crc16_modbus(bytes_hex) # Retorna o CRC

            parte_superior = (crc_result >> 8) & 0xFF  # Desloca 8 bits para a direita e aplica a máscara 0xFF
            parte_inferior = crc_result & 0xFF        # Aplica a máscara 0xFF diretamente

            superior_crc = int(dados_recebidos[14:16],16) # Transforma de hexa para int
            inferior_crc = int(dados_recebidos[12:14],16) # Transforma de hexa para int

            if parte_superior == superior_crc and parte_inferior == inferior_crc:
                dados_recebidos = dados_recebidos[14:16]
                dados_recebidos = int(dados_recebidos,16)
                # time.sleep(0.5)
                return dados_recebidos
            else:
                return -1
        except:
            return -1 # Indica erro de alguma natureza....
    def broadcast_config_serial(self, adr=1, config=2):
        
        dados_recebidos = None

        id_loc = adr
        config_loc = config

        id_loc = str(id_loc)
        config_loc = str(config_loc)
        id_loc = id_loc.zfill(2).upper()
        config_loc = config_loc.zfill(4).upper()

        hex_text = f"{id_loc}060065{config_loc}"

        bytes_hex = bytes.fromhex(hex_text) # Transforma em hexa

        crc_result = self.crc16_modbus(bytes_hex) # Retorna o CRC

        parte_superior = (crc_result >> 8) & 0xFF  # Desloca 8 bits para a direita e aplica a máscara 0xFF
        parte_inferior = crc_result & 0xFF        # Aplica a máscara 0xFF diretamente


        # Repete-se os comandos em decimal com os devidos bytes de CRC
        self.ser.write([adr, 6,0,0x65,0,config,parte_inferior,parte_superior])

        while self.ser.readable()==False:
            pass
        dados_recebidos = self.ser.read(8)
        try:
            dados_recebidos = dados_recebidos.hex()
            hex_text = dados_recebidos[0:2]+dados_recebidos[2:4]+dados_recebidos[4:6]+dados_recebidos[6:8]+dados_recebidos[8:10]+dados_recebidos[10:12]
            bytes_hex = bytes.fromhex(hex_text) # Transforma em hexa
            crc_result = self.crc16_modbus(bytes_hex) # Retorna o CRC

            parte_superior = (crc_result >> 8) & 0xFF  # Desloca 8 bits para a direita e aplica a máscara 0xFF
            parte_inferior = crc_result & 0xFF        # Aplica a máscara 0xFF diretamente

            superior_crc = int(dados_recebidos[14:16],16) # Transforma de hexa para int
            inferior_crc = int(dados_recebidos[12:14],16) # Transforma de hexa para int

            if parte_superior == superior_crc and parte_inferior == inferior_crc:
                dados_recebidos = dados_recebidos[14:16]
                dados_recebidos = int(dados_recebidos,16)
                # time.sleep(0.5)

                baund = self._config_serial(confi=config)
                
                return adr,baund
            else:
                return -1, -1
        except:
            return -1, -1 # Indica erro de alguma natureza....
        
    def adam_wp9038_out(self,id = 1, out=[0,0,0,0]):
        
        dados_recebidos = None
        # self.in_out.re_de_485(self.in_out.SEND_485)

        id_loc = hex(id)[2:]
        id_loc = id_loc.zfill(2).upper()

        out = list(reversed(out))

        out_loc = ''.join(str(bit) for bit in out)
        out_loc = int(out_loc, 2)
        out_val = out_loc

        out_loc = hex(out_loc)[2:]
        out_loc = out_loc.zfill(2).upper()

        hex_text = f"{id_loc} 0f 00 00 00 04 01 {out_loc}"
        bytes_hex = bytes.fromhex(hex_text) # Transforma em hexa

        crc_result = self.crc16_modbus(bytes_hex) # Retorna o CRC

        parte_superior = (crc_result >> 8) & 0xFF  # Desloca 8 bits para a direita e aplica a máscara 0xFF
        parte_inferior = crc_result & 0xFF        # Aplica a máscara 0xFF diretamente

        id_loc = id

        # Repete-se os comandos em decimal com os devidos bytes de CRC
        # self._serial.flush()
        self.ser.write([id_loc,0x0f,0,0,0,4,1,out_val,parte_inferior,parte_superior])
        # self._serial.flush()
        # self.in_out.re_de_485(self.in_out.RECV_485)
        # sleep(0.2)

        while self.ser.readable()==False:
            pass
        dados_recebidos = self.ser.read(8)
        try:
            dados_recebidos = dados_recebidos.hex()
            hex_text = dados_recebidos[0:2]+dados_recebidos[2:4]+dados_recebidos[4:6]+dados_recebidos[6:8]+dados_recebidos[8:10]+dados_recebidos[10:12]
            bytes_hex = bytes.fromhex(hex_text) # Transforma em hexa
            crc_result = self.crc16_modbus(bytes_hex) # Retorna o CRC

            parte_superior = (crc_result >> 8) & 0xFF  # Desloca 8 bits para a direita e aplica a máscara 0xFF
            parte_inferior = crc_result & 0xFF        # Aplica a máscara 0xFF diretamente

            superior_crc = int(dados_recebidos[14:16],16) # Transforma de hexa para int
            inferior_crc = int(dados_recebidos[12:14],16) # Transforma de hexa para int

            if parte_superior == superior_crc and parte_inferior == inferior_crc:
                dados_recebidos = dados_recebidos[14:16]
                dados_recebidos = int(dados_recebidos)
                # time.sleep(0.5)
                return dados_recebidos
            else:
                return -1
        except:
            return -1 # Indica erro de alguma natureza....
        
    def _config_serial(self, confi=2):
        
        self.ser.close()
        if confi == 1:
            self.ser.baudrate = 4800
            self.ser.stopbits = 1
            self.ser.bytesize = 8
            self.ser.parity = "N"
        elif confi == 2:
            self.ser.baudrate = 9600
            self.ser.stopbits = 1
            self.ser.bytesize = 8
            self.ser.parity = "N"
        elif confi == 3:
            self.ser.baudrate = 19200
            self.ser.stopbits = 1
            self.ser.bytesize = 8
            self.ser.parity = "N"
        elif confi == 4:
            self.ser.baudrate = 38400
            self.ser.stopbits = 1
            self.ser.bytesize = 8
            self.ser.parity = "N"
        elif confi == 5:
            self.ser.baudrate = 4800
            self.ser.stopbits = 1
            self.ser.bytesize = 8
            self.ser.parity = "E"
        elif confi == 6:
            self.ser.baudrate = 9600
            self.ser.stopbits = 1
            self.ser.bytesize = 8
            self.ser.parity = "E"
        elif confi == 7:
            self.ser.baudrate = 19200
            self.ser.stopbits = 1
            self.ser.bytesize = 8
            self.ser.parity = "E"
        elif confi == 8:
            self.ser.baudrate = 38400
            self.ser.stopbits = 1
            self.ser.bytesize = 8
            self.ser.parity = "E"

        self.ser.open()
        return self.ser.baudrate


'''
0001- 4800, None, 8, 1
0002- 9600, None, 8, 1
0003- 19200, None, 8, 1
0004- 38400, None, 8, 1
0005- 4800, Even, 8, 1
0006- 9600, Even, 8, 1
0007- 19200, Even, 8, 1
0008- 38400, Even, 8, 1
'''

if __name__ == '__main__':
    mod = RTU485_Config()
    ret = None
    cmd =""
    cmd = input(f"Qual configuração de porta deseja iniciar?\nVer tabela do dispositivo...\n")
    baund_ = mod._config_serial(confi=int(cmd))
    print(f"Baundrate do sistema em {baund_}.\n")

    while cmd.upper() != "Q":
        cmd = input(f"Digite o comando desejado.\n1 - Para Confifurar endereço.\n2 - Para configurar porta serial.\n3 - Para testar módulos.\n")
        if cmd.upper() == "1":
            id_l = input(f"Certifique-se que só há o dispositivo a ser configurado.\nDigite o endereço que deseja associar ao dispositivo.\n")
            ret = mod.broadcast_config_address(adr=int(id_l))
            print(f"Dispositivo configurado para endereço: {int(id_l)}\nCódigo validação: {ret}\n")
        elif cmd.upper() == "2":
            adr_l = input(f"Digite o endereço do dispositivo para configurar porta serial.\n")
            conf_l = input(f"Digite a configuração conforme tabela.\n")
            ret = mod.broadcast_config_serial(adr=int(adr_l), config=int(conf_l))
            print(f"Dispositivo configurado para endereço: {int(adr_l)} com baundrate de : {ret}\n")
        elif cmd.upper() == "3":
            cmd = input(f"Digite qual dispositivo deseja testar.\n1 - para WP9038ADAM.\n2 - Para WP8025ADAM.\n3 - Para WP8026ADAM.\n4 - Para WP8027ADAM.\n")
            if cmd == "1":
                id_l = input(f"Digite o Endereço do dispositivo.\n")
                cmd = input(f"Digitar saída no modelo x,x,x,x. Ex.: 0,1,1,0 acionará as saídas DO_04,DO_03,DO_02 e DO_01.\n")
                lista = [ int(x) for x in cmd.split(',') ]
                mod.adam_wp9038_out( id=int(id_l), out=lista)
    mod.close()