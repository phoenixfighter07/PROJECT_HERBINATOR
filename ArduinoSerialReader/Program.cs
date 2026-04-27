using System;
using System.IO.Ports;

namespace ArduinoConsole
{
    class ArduinoSerialReader()
    {
        static List<string> data = new List<string>();
        /*
            Method to initialize com port and write to list each string that comes in.
        */
        static void Main(String[] args)
        {
            string comPort = "COM3";
            int baud = 9600;
            SerialPort sp = new SerialPort(comPort, baud);

            sp.DtrEnable = true;
            sp.RtsEnable = true;
            sp.DataReceived += new SerialDataReceivedEventHandler(ShowArduinoMsg);
            sp.Open();

            Console.WriteLine("Press any key to stop...");
            Console.WriteLine();
            Console.ReadKey();
            Console.WriteLine("\nPort terminated!");
            File.AppendAllLines(Environment.GetFolderPath(Environment.SpecialFolder.Desktop) + "//LogFile.csv", data);

            sp.Close();
        }
        /*
            ShowArduinoMsg
            - sender: The object that sent the information, in this case a SerialPort object.
            - e: The recieved event arguments from the com port.
        */
        public static void ShowArduinoMsg(object sender, SerialDataReceivedEventArgs e)
        {
            SerialPort sp = (SerialPort) sender;
            string indata = sp.ReadLine();
            Console.WriteLine(indata);
            data.Add(indata.Trim('\r'));
        }
    }
}