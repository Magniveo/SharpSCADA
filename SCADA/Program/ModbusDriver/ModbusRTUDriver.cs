using DataService;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.IO.Ports;
using System.Text;
using System.Threading;

namespace ModbusDriver
{
    [Description("Modbus RTU protocol")]
    //ModbusRTUReader : IPLCDriver         IPLCDriver : IDriver, IReaderWriter              IDriver : IDisposable
    public sealed class ModbusRTUReader : IPLCDriver
    {
        #region :IDriver
        //Slave address
        short _id;
        public short ID
        {
            get
            {
                return _id;
            }
        }

        string _name;
        public string Name
        {
            get
            {
                return _name;
            }
        }

        string _port = "COM1";
        [Category("Serial port settings"), Description("Serial number")]
        public string PortName
        {
            get { return _port; }
            set { _port = value; }
        }

        public bool IsClosed
        {
            get
            {
                return _serialPort == null || _serialPort.IsOpen == false;
            }
        }

        private int _timeOut = 3000;
        [Category("Serial port settings"), Description("Communication timeout")]
        public int TimeOut
        {
            get { return _timeOut; }
            set { _timeOut = value; }
        }


        private int _baudRate = 9600;
        [Category("Serial port settings"), Description("波特率")]
        public int BaudRate
        {
            get { return _baudRate; }
            set { _baudRate = value; }
        }
        //   private SerialPort _serialPort;

        private int _dataBits = 8;
        [Category("Serial port settings"), Description("Data bit")]
        public int DataBits
        {
            get { return _dataBits; }
            set { _dataBits = value; }
        }
        private StopBits _stopBits = StopBits.One;
        [Category("Serial port settings"), Description("Stop bit")]
        public StopBits StopBits
        {
            get { return _stopBits; }
            set { _stopBits = value; }
        }

        private Parity _parity = Parity.None;
        [Category("Serial port settings"), Description("Parity check")]
        public Parity Parity
        {
            get { return _parity; }
            set { _parity = value; }
        }

        List<IGroup> _grps = new List<IGroup>();
        public IEnumerable<IGroup> Groups
        {
            get { return _grps; }
        }

        IDataServer _server;
        public IDataServer Parent
        {
            get { return _server; }
        }

        public bool Connect()
        {
            try
            {
                if (_timeOut <= 0) _timeOut = 1000;
                if (_serialPort == null)
                    _serialPort = new SerialPort(_port);
                _serialPort.ReadTimeout = _timeOut;
                _serialPort.WriteTimeout = _timeOut;
                _serialPort.BaudRate = _baudRate;
                _serialPort.DataBits = _dataBits;
                _serialPort.Parity = _parity;
                _serialPort.StopBits = _stopBits;
                _serialPort.Open();
                return true;
            }
            catch (IOException error)
            {
                if (OnError != null)
                {
                    OnError(this, new IOErrorEventArgs(error.Message));
                }
                return false;
            }
        }

        public IGroup AddGroup(string name, short id, int updateRate, float deadBand = 0f, bool active = false)
        {
            ShortGroup grp = new ShortGroup(id, name, updateRate, active, this);
            _grps.Add(grp);
            return grp;
        }

        public bool RemoveGroup(IGroup grp)
        {
            grp.IsActive = false;
            return _grps.Remove(grp);
        }
        public event IOErrorEventHandler OnError;
        #endregion
        //Custom constructor 3
        public ModbusRTUReader(IDataServer server, short id, string name)
        {
            _id = id;
            _name = name;
            _server = server;
        }

        private SerialPort _serialPort;

        /*
         Sbyte: Represents a signed 8-bit integer with a value ranging from -128 to 127
　       Byte: represents an unsigned 8-bit integer, the value range is from 0 to 255
　       Short: represents a signed 16-bit integer, ranging from -32768 to 32767
　       ushort: represents a signed 16-bit integer, ranging from 0 to 65,535
         Int: Represents a signed 32-bit integer, ranging from -2147483648 to 2147483648
　　     Uint: represents an unsigned 32-bit integer, ranging from 0 to 4294967295
　       Long: represents a signed 64-bit integer, ranging from -9223372036854775808 to 9223372036854775808
　       Ulong: represents an unsigned 64-bit integer ranging from 0 to 18446744073709551615.
        */
        private byte[] CreateReadHeader(int id, int startAddress, ushort length, byte function)
        {
            byte[] data = new byte[8];
            data[0] = (byte)id;				// Slave id high byte   Slave address
            data[1] = function;					// Message size
            byte[] _adr = BitConverter.GetBytes((short)startAddress);//Returns the specified as an array of bytes 16 Bit unsigned integer value。
            //apply on small endian, TODO:support big endian
            data[2] = _adr[1];				// Start address The upper eight bits of the starting address
            data[3] = _adr[0];				// Start address The lower eight bits of the starting address
            byte[] _length = BitConverter.GetBytes((short)length);
            //apply on small endian, TODO:support big endian
            data[4] = _length[1];			// Number of data to read The upper eight bits of the number of registers
            data[5] = _length[0];			// Number of data to read The lower eight bits of the number of registers
            byte[] arr = Utility.CalculateCrc(data, 6);
            data[6] = arr[0]; //The lower eight bits of CRC check
            data[7] = arr[1];//The high eight bits of CRC check
            return data;
        }

        #region Write a single coil or a single discrete output function code：0x05
        public int WriteSingleCoils(int id, int startAddress, bool OnOff)
        {
            byte[] data = new byte[8];
            data[0] = (byte)id;				// Slave id high byte
            data[1] = Modbus.fctWriteSingleCoil;				// Function code
            byte[] _adr = BitConverter.GetBytes((short)startAddress);
            data[2] = _adr[1];				// Start address
            data[3] = _adr[0];              // Start address
            if (OnOff) data[4] = 0xFF;
            byte[] arr = Utility.CalculateCrc(data, 6);
            data[6] = arr[0];
            data[7] = arr[1];
            lock (_async)
            {
                _serialPort.Write(data, 0, data.Length);
                int numBytesRead = 0;
                var frameBytes = new byte[5];
                while (numBytesRead != frameBytes.Length)
                    numBytesRead += _serialPort.Read(frameBytes, numBytesRead, frameBytes.Length - numBytesRead);
                var slave = frameBytes[0];
                var code = frameBytes[1];
                if (code == 0x85)//Errors only need to read 5 bytes
                {
                    var errorcode = frameBytes[2];
                    if (OnError != null)
                    {
                        OnError(this, new IOErrorEventArgs(Modbus.GetErrorString(errorcode)));
                    }
                    Thread.Sleep(10);
                    return -1;
                }
                else//8 bytes is required
                {
                    numBytesRead = 0;
                    while (numBytesRead < 3)
                        numBytesRead += _serialPort.Read(frameBytes, numBytesRead, 3 - numBytesRead);
                    Thread.Sleep(10);
                    return 0;
                }
            }
        }
        #endregion

        #region Write multiple coil function code: 0x0F  15
        public int WriteMultipleCoils(int id, int startAddress, ushort numBits, byte[] values)
        {
            int len = values.Length;
            byte[] data = new byte[len + 9];
            data[0] = (byte)id;				// Slave id high byte  Slave address High eight
            data[1] = Modbus.fctWriteMultipleCoils;				// Function code  function code
            byte[] _adr = BitConverter.GetBytes((short)startAddress);
            data[2] = _adr[1];				// Start address       Start address high eight
            data[3] = _adr[0];				// Start address       The lower eight bits of the start address
            byte[] _length = BitConverter.GetBytes((short)numBits);
            data[4] = _length[1];			// Number of data to read  The upper eight bits of the number of registers
            data[5] = _length[0];           // Number of data to read  The lower eight registers
            data[6] = (byte)len;            //Bytes the amount
            Array.Copy(values, 0, data, 7, len);  //Add change data to data
            byte[] arr = Utility.CalculateCrc(data, len + 7);
            data[len + 7] = arr[0];  //The lower eight bits of CRC check
            data[len + 8] = arr[1];  //The high eight bits of CRC check
            lock (_async)
            {
                _serialPort.Write(data, 0, data.Length);
                int numBytesRead = 0;
                var frameBytes = new byte[5];
                while (numBytesRead != frameBytes.Length)
                    numBytesRead += _serialPort.Read(frameBytes, numBytesRead, frameBytes.Length - numBytesRead);
                var slave = frameBytes[0];
                var code = frameBytes[1];
                if (code == 0x85)//Errors only need to read 5 bytes
                {
                    var errorcode = frameBytes[2];
                    if (OnError != null)
                    {
                        OnError(this, new IOErrorEventArgs(Modbus.GetErrorString(errorcode)));
                    }
                    Thread.Sleep(10);
                    return -1;
                }
                else//9 bytes is required
                {
                    numBytesRead = 0;
                    while (numBytesRead < 4)
                        numBytesRead += _serialPort.Read(frameBytes, numBytesRead, 3 - numBytesRead);
                    Thread.Sleep(10);
                    return 0;
                }
            }
        }
        #endregion

        #region Write a single holding register function code:0x06
        public int WriteSingleRegister(int id, int startAddress, byte[] values)
        {
            byte[] data = new byte[8];
            data[0] = (byte)id;				// Slave id high byte Slave address High eight
            data[1] = Modbus.fctWriteSingleRegister;				// Function code function code
            byte[] _adr = BitConverter.GetBytes((short)startAddress);
            data[2] = _adr[1];				// Start address    Start address high eight
            data[3] = _adr[0];				// Start address    Start address high eight
            data[4] = values[1];            //Change the high bit of the data
            data[5] = values[0];            //Change the low bit of data
            byte[] arr = Utility.CalculateCrc(data, 6);
            data[6] = arr[0];               //The lower eight bits of the CRC check code
            data[7] = arr[1];               //The upper eight bits of the CRC check code
            lock (_async)
            {
                _serialPort.Write(data, 0, data.Length);
                int numBytesRead = 0;
                var frameBytes = new byte[5];
                while (numBytesRead != frameBytes.Length)
                    numBytesRead += _serialPort.Read(frameBytes, numBytesRead, frameBytes.Length - numBytesRead);
                var slave = frameBytes[0];
                var code = frameBytes[1];
                if (code == 0x85)//Errors only need to read 5 bytes
                {
                    var errorcode = frameBytes[2];
                    if (OnError != null)
                    {
                        OnError(this, new IOErrorEventArgs(Modbus.GetErrorString(errorcode)));
                    }
                    Thread.Sleep(10);
                    return -1;
                }
                else//8 bytes is required
                {
                    numBytesRead = 0;
                    while (numBytesRead < 3)
                        numBytesRead += _serialPort.Read(frameBytes, numBytesRead, 3 - numBytesRead);
                    Thread.Sleep(10);
                    return 0;
                }
            }
        }
        #endregion

        #region Write multiple holding registers function code：0x10    16
        public int WriteMultipleRegister(int id, int startAddress, byte[] values)
        {
            int len = values.Length;
            if (len % 2 > 0) len++;
            byte[] data = new byte[len + 9];
            data[0] = (byte)id;			// Slave id high byte  Slave address
            data[1] = Modbus.fctWriteMultipleRegister;				// Function code  function code
            byte[] _adr = BitConverter.GetBytes((short)startAddress);
            data[2] = _adr[1];				// Start address        Start address high eight
            data[3] = _adr[0];				// Start address        The lower eight bits of the start address
            byte[] _length = BitConverter.GetBytes((short)(len >> 1));
            data[4] = _length[1];			// Number of data to read The upper eight bits of the number of registers
            data[5] = _length[0];			// Number of data to read The lower eight registers
            data[6] = (byte)len;            //Bytes
            Array.Copy(values, 0, data, 7, len); //Add change data to data
            byte[] arr = Utility.CalculateCrc(data, len + 7);
            data[len + 7] = arr[0];          //The lower eight bits of CRC check
            data[len + 8] = arr[1];          //The high eight bits of CRC check
            lock (_async)
            {
                _serialPort.Write(data, 0, data.Length);
                int numBytesRead = 0;
                var frameBytes = new byte[5];
                while (numBytesRead != frameBytes.Length)
                    numBytesRead += _serialPort.Read(frameBytes, numBytesRead, frameBytes.Length - numBytesRead);
                var slave = frameBytes[0];
                var code = frameBytes[1];
                if (code == 0x85)//Errors only need to read 5 bytes
                {
                    var errorcode = frameBytes[2];
                    if (OnError != null)
                    {
                        OnError(this, new IOErrorEventArgs(Modbus.GetErrorString(errorcode)));
                    }
                    Thread.Sleep(10);
                    return -1;
                }
                else//8 bytes is required
                {
                    numBytesRead = 0;
                    while (numBytesRead < 3)
                        numBytesRead += _serialPort.Read(frameBytes, numBytesRead, 3 - numBytesRead);
                    Thread.Sleep(10);
                    return 0;
                }
            }
        }
        #endregion

        #region  :IPLCDriver
        public int PDU
        {
            // get { return 0x100; } //0x100 decimal value is 256
            /* Updated by: yjz
                Update date: 20171125
                Reason for update: The RS232 / RS485 modbus protocol in serial communication is as follows:
                         ADU=address field+function code+data+error check Among them ADU 256 bytes, address field 1 byte, function code1 byte, data is 252 bytes, error check 2 bytes,
                         PDU=function code+data
                         So the PDU should be: 253 bytes */
            get { return 0xFD; } //0xFD 253 decimal
        }
        private string _serverName = "unknown";
        public string ServerName
        {
            get { return _serverName; }
            set { _serverName = value; }
        }

        public DeviceAddress GetDeviceAddress(string address)
        {
            DeviceAddress dv = DeviceAddress.Empty;
            if (string.IsNullOrEmpty(address))
                return dv;
            var sindex = address.IndexOf(':');
            if (sindex > 0)
            {
                int slaveId;
                if (int.TryParse(address.Substring(0, sindex), out slaveId))
                    dv.Area = slaveId;
                address = address.Substring(sindex + 1);
            }
            switch (address[0])
            {
                case '0':
                    {
                        dv.DBNumber = Modbus.fctReadCoil;
                        int st;
                        int.TryParse(address, out st);
                        dv.Bit = (byte)(st % 16);
                        st /= 16;
                        dv.Start = st;
                        dv.Bit--;
                    }
                    break;
                case '1':
                    {
                        dv.DBNumber = Modbus.fctReadDiscreteInputs;
                        int st;
                        int.TryParse(address.Substring(1), out st);
                        dv.Bit = (byte)(st % 16);
                        st /= 16;
                        dv.Start = st;
                        dv.Bit--;
                    }
                    break;
                case '4':
                    {
                        int index = address.IndexOf('.');
                        dv.DBNumber = Modbus.fctReadHoldingRegister;
                        if (index > 0)
                        {
                            dv.Start = int.Parse(address.Substring(1, index - 1));
                            dv.Bit = byte.Parse(address.Substring(index + 1));
                        }
                        else
                            dv.Start = int.Parse(address.Substring(1));
                        dv.Start--;
                        dv.Bit--;
                        dv.ByteOrder = ByteOrder.BigEndian;
                    }
                    break;
                case '3':
                    {
                        int index = address.IndexOf('.');
                        dv.DBNumber = Modbus.fctReadInputRegister;
                        if (index > 0)
                        {
                            dv.Start = int.Parse(address.Substring(1, index - 1));
                            dv.Bit = byte.Parse(address.Substring(index + 1));
                        }
                        else
                            dv.Start = int.Parse(address.Substring(1));
                        dv.Start--;
                        dv.Bit--;
                        dv.ByteOrder = ByteOrder.BigEndian;
                    }
                    break;
            }
            return dv;
        }

        public string GetAddress(DeviceAddress address)
        {
            return string.Empty;
        }
        #endregion

        #region :IReaderWriter 
        object _async = new object();
        public byte[] ReadBytes(DeviceAddress address, ushort size)
        {
            var func = (byte)address.DBNumber;
            try
            {
                byte[] header = func < 3 ? CreateReadHeader(address.Area, address.Start * 16, (ushort)(16 * size), func) :
                 CreateReadHeader(address.Area, address.Start, size, func);
                lock (_async)
                {
                    byte[] frameBytes = new byte[size * 2 + 5];//size * 2 +
                    byte[] data = new byte[size * 2];
                    _serialPort.Write(header, 0, header.Length);
                    int numBytesRead = 0;
                    while (numBytesRead < 2)
                        numBytesRead += _serialPort.Read(frameBytes, numBytesRead, 2 - numBytesRead);
                    if (frameBytes[1] == address.DBNumber)
                    {
                        while (numBytesRead < frameBytes.Length)
                            numBytesRead += _serialPort.Read(frameBytes, numBytesRead, frameBytes.Length - numBytesRead);
                        if (Utility.CheckSumCRC(frameBytes))
                        {
                            Array.Copy(frameBytes, 3, data, 0, data.Length);
                            Thread.Sleep(20);
                            return data;
                        }
                        else Thread.Sleep(10);
                    }
                    else
                    {
                        numBytesRead = 0;
                        while (numBytesRead < 3)
                            numBytesRead += _serialPort.Read(frameBytes, numBytesRead, 3 - numBytesRead);
                        Thread.Sleep(10);
                    }

                }
                return null;
            }
            catch (Exception e)
            {
                if (OnError != null)
                    OnError(this, new IOErrorEventArgs(e.Message));
                return null;
            }
        }

        public ItemData<int> ReadInt32(DeviceAddress address)
        {
            byte[] bit = ReadBytes(address, 2);
            return bit == null ? new ItemData<int>(0, 0, QUALITIES.QUALITY_BAD) :
                new ItemData<int>(BitConverter.ToInt32(bit, 0), 0, QUALITIES.QUALITY_GOOD);
        }

        public ItemData<uint> ReadUInt32(DeviceAddress address)
        {
            byte[] bit = ReadBytes(address, 2);
            return bit == null ? new ItemData<uint>(0, 0, QUALITIES.QUALITY_BAD) :
                new ItemData<uint>(BitConverter.ToUInt32(bit, 0), 0, QUALITIES.QUALITY_GOOD);
        }

        public ItemData<ushort> ReadUInt16(DeviceAddress address)
        {
            byte[] bit = ReadBytes(address, 1);
            return bit == null ? new ItemData<ushort>(0, 0, QUALITIES.QUALITY_BAD) :
                new ItemData<ushort>(BitConverter.ToUInt16(bit, 0), 0, QUALITIES.QUALITY_GOOD);
        }

        public ItemData<short> ReadInt16(DeviceAddress address)
        {
            byte[] bit = ReadBytes(address, 1);
            return bit == null ? new ItemData<short>(0, 0, QUALITIES.QUALITY_BAD) :
                new ItemData<short>(BitConverter.ToInt16(bit, 0), 0, QUALITIES.QUALITY_GOOD);
        }

        public ItemData<byte> ReadByte(DeviceAddress address)
        {
            byte[] bit = ReadBytes(address, 1);
            return bit == null ? new ItemData<byte>(0, 0, QUALITIES.QUALITY_BAD) :
                 new ItemData<byte>(bit[0], 0, QUALITIES.QUALITY_GOOD);
        }

        public ItemData<string> ReadString(DeviceAddress address, ushort size)
        {
            byte[] bit = ReadBytes(address, size);
            return bit == null ? new ItemData<string>(string.Empty, 0, QUALITIES.QUALITY_BAD) :
                new ItemData<string>(Encoding.ASCII.GetString(bit), 0, QUALITIES.QUALITY_GOOD);
        }

        public ItemData<float> ReadFloat(DeviceAddress address)
        {
            byte[] bit = ReadBytes(address, 2);
            return bit == null ? new ItemData<float>(0f, 0, QUALITIES.QUALITY_BAD) :
                new ItemData<float>(BitConverter.ToSingle(bit, 0), 0, QUALITIES.QUALITY_GOOD);
        }

        public ItemData<bool> ReadBit(DeviceAddress address)
        {
            byte[] bit = ReadBytes(address, 1);
            return bit == null ? new ItemData<bool>(false, 0, QUALITIES.QUALITY_BAD) :
                 new ItemData<bool>((bit[0] & (1 << (address.Bit))) > 0, 0, QUALITIES.QUALITY_GOOD);
        }

        public ItemData<object> ReadValue(DeviceAddress address)
        {
            return this.ReadValueEx(address);
        }

        public int WriteBytes(DeviceAddress address, byte[] bit)
        {
            return WriteMultipleRegister(address.Area, address.Start, bit);
        }

        public int WriteBit(DeviceAddress address, bool bit)
        {
            return WriteSingleCoils(address.Area, address.Start + address.Bit, bit);
        }

        public int WriteBits(DeviceAddress address, byte bits)
        {
            return WriteSingleRegister(address.Area, address.Start, new byte[] { bits });
        }

        public int WriteInt16(DeviceAddress address, short value)
        {
            return WriteSingleRegister(address.Area, address.Start, BitConverter.GetBytes(value));
        }

        public int WriteUInt16(DeviceAddress address, ushort value)
        {
            return WriteSingleRegister(address.Area, address.Start, BitConverter.GetBytes(value));
        }

        public int WriteUInt32(DeviceAddress address, uint value)
        {
            return WriteMultipleRegister(address.Area, address.Start, BitConverter.GetBytes(value));
        }

        public int WriteInt32(DeviceAddress address, int value)
        {
            return WriteMultipleRegister(address.Area, address.Start, BitConverter.GetBytes(value));
        }

        public int WriteFloat(DeviceAddress address, float value)
        {
            return WriteMultipleRegister(address.Area, address.Start, BitConverter.GetBytes(value));
        }

        public int WriteString(DeviceAddress address, string str)
        {
            return WriteMultipleRegister(address.Area, address.Start, Encoding.ASCII.GetBytes(str));
        }

        public int WriteValue(DeviceAddress address, object value)
        {
            return this.WriteValueEx(address, value);
        }

        #endregion

        #region : IDisposable

        public void Dispose()
        {
            foreach (IGroup grp in _grps)
            {
                grp.Dispose();
            }
            _grps.Clear();
            _serialPort.Close();
        }
        #endregion
    }

    public sealed class Modbus
    {
        public const byte fctReadCoil = 1;
        public const byte fctReadDiscreteInputs = 2;
        public const byte fctReadHoldingRegister = 3;
        public const byte fctReadInputRegister = 4;
        public const byte fctWriteSingleCoil = 5;
        public const byte fctWriteSingleRegister = 6;
        public const byte fctWriteMultipleCoils = 15;
        public const byte fctWriteMultipleRegister = 16;
        public const byte fctReadWriteMultipleRegister = 23;

        /// <summary>Constant for exception illegal function.</summary>
        public const byte excIllegalFunction = 1;
        /// <summary>Constant for exception illegal data address.</summary>
        public const byte excIllegalDataAdr = 2;
        /// <summary>Constant for exception illegal data value.</summary>
        public const byte excIllegalDataVal = 3;
        /// <summary>Constant for exception slave device failure.</summary>
        public const byte excSlaveDeviceFailure = 4;
        /// <summary>Constant for exception acknowledge.</summary>
        public const byte excAck = 5;
        /// <summary>Constant for exception slave is busy/booting up.</summary>
        public const byte excSlaveIsBusy = 6;
        /// <summary>Constant for exception gate path unavailable.</summary>
        public const byte excGatePathUnavailable = 10;
        /// <summary>Constant for exception not connected.</summary>
        public const byte excExceptionNotConnected = 253;
        /// <summary>Constant for exception connection lost.</summary>
        public const byte excExceptionConnectionLost = 254;
        /// <summary>Constant for exception response timeout.</summary>
        public const byte excExceptionTimeout = 255;
        /// <summary>Constant for exception wrong offset.</summary>
        public const byte excExceptionOffset = 128;
        /// <summary>Constant for exception send failt.</summary>
        public const byte excSendFailt = 100;

        public static string GetErrorString(byte exception)
        {
            switch (exception)
            {
                case Modbus.excIllegalFunction:
                    return "Constant for ModbusModbus.exception illegal function.";
                case Modbus.excIllegalDataAdr:
                    return "Constant for ModbusModbus.exception illegal data address.";
                case Modbus.excIllegalDataVal:
                    return "Constant for ModbusModbus.exception illegal data value.";
                case Modbus.excSlaveDeviceFailure:
                    return "Constant for ModbusModbus.exception slave device failure.";
                case Modbus.excAck:
                    return "Constant for ModbusModbus.exception acknowledge.";
                case Modbus.excSlaveIsBusy:
                    return "Constant for ModbusModbus.exception slave is busy/booting up.";
                case Modbus.excGatePathUnavailable:
                    return "Constant for ModbusModbus.exception gate path unavailable.";
                case Modbus.excExceptionNotConnected:
                    return "Constant for ModbusModbus.exception not connected.";
                case Modbus.excExceptionConnectionLost:
                    return "Constant for ModbusModbus.exception connection lost.";
                case Modbus.excExceptionTimeout:
                    return "Constant for ModbusModbus.exception response timeout.";
                case Modbus.excExceptionOffset:
                    return "Constant for ModbusModbus.exception wrong offset.";
                case Modbus.excSendFailt:
                    return "Constant for ModbusModbus.exception send failt.";
            }
            return string.Empty;
        }
    }
}
