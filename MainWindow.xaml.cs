using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Globalization;
using System.IO;
using System.IO.Ports;
using System.Text;
using System.Threading;
using System.Windows;
using System.Windows.Controls;
using Microsoft.Win32;
using OxyPlot;
using OxyPlot.Series;
using OxyPlot.Axes;
using OxyPlot.Wpf;

namespace HMI
{
    public partial class MainWindow : Window
    {
        // 串口通信相关变量
        private SerialPort serialPort;
        private Queue<byte[]> dataQueue = new Queue<byte[]>(); // 数据接收队列
        private object queueLock = new object(); // 队列锁
        private byte[] receiveBuffer = new byte[8192]; // 接收缓冲区
        private volatile bool isReceiving = false; // 接收线程标志
        private volatile bool isProcessing = false; // 处理线程标志
        private Thread receiveThread; // 接收线程
        private Thread processingThread; // 数据处理线程

        // 数据存储与保存相关变量
        private volatile bool isSave1HzEnabled = false; // 1Hz数据保存标志
        private volatile bool isSave100HzEnabled = false; // 100Hz数据保存标志
        private string filePath1Hz = string.Empty; // 1Hz数据文件路径
        private string filePath100Hz = string.Empty; // 100Hz数据文件路径
        private StreamWriter writer1Hz = null; // 1Hz数据写入流
        private StreamWriter writer100Hz = null; // 100Hz数据写入流
        private CancellationTokenSource cancellationTokenSource; // 用于取消线程

        // 数据处理与绘图相关变量
        private Dictionary<string, LineSeries> lineSeries = new Dictionary<string, LineSeries>(); // 绘图数据系列
        private Dictionary<string, List<double>> buffer100HzByParam = new Dictionary<string, List<double>>(); // 100Hz数据缓冲区
        private Dictionary<string, List<double>> buffer1HzByParams = new Dictionary<string, List<double>>(); // 所有参数数据缓冲区
        private volatile int Counter_100Hz = 0; // 100Hz计数器
        private volatile int Counter_1Hz = 0; // 1Hz计数器
        private int linesWritten1Hz = 0; // 1Hz数据写入行数
        private DateTime lastUIUpdate = DateTime.Now; // 上次UI更新时间
        private const int UI_UPDATE_INTERVAL = 1000; // UI更新间隔（毫秒）
        private bool isDraw100HzSelected = false; // 100Hz绘图选择标志
        private bool isDraw1HzSelected = false; // 1Hz绘图选择标志

        // 参数与控件映射相关变量
        private Dictionary<string, TextBox> textBoxMap = new Dictionary<string, TextBox>(); // 参数与TextBox映射
        private List<string> plotParameters = new List<string> { "a", "q", "theta", "delta", "f", "Omega" }; // 需要绘图的参数
        private Dictionary<string, string> formatRules = new Dictionary<string, string>(); // 参数格式化规则
        private StringBuilder fifoBuffer = new StringBuilder(); // 数据接收缓冲区
        private int sampleTime = 3600; // 采样时间（秒）
        private DateTime startTime; // 开始时间

        // PlotModel集合
        private Dictionary<string, PlotModel> plotModels = new Dictionary<string, PlotModel>();

        // 构造函数
        public MainWindow()
        {
            InitializeComponent();
            this.Closing += MainWindow_Closing; // 注册窗口关闭事件
            InitializeControls(); // 初始化控件
            InitializeSerialPort(); // 初始化串口
            InitializePlot(); // 初始化绘图
            InitializeTextBoxMap(); // 初始化参数与控件映射
            InitializeFormatRules(); // 初始化格式化规则
            StartReceiveThread(); // 启动接收线程
            StartProcessingThread(); // 启动处理线程
        }

        // 初始化控件
        private void InitializeControls()
        {
            // 设置串口名称下拉框
            string[] portNames = SerialPort.GetPortNames();
            com.ItemsSource = portNames;
            com.SelectedIndex = 0;

            // 设置波特率下拉框
            string[] baudRates = { "9600", "19200", "38400", "57600", "115200", "460800" };
            Baud_rate.ItemsSource = baudRates;
            Baud_rate.SelectedIndex = 5;

            // 设置默认采样时间
            sample_time.Text = "3600";
        }

        // 初始化串口
        private void InitializeSerialPort()
        {
            serialPort = new SerialPort
            {
                Parity = Parity.None,
                StopBits = StopBits.One,
                DataBits = 8,
                Handshake = Handshake.None,
                ReadBufferSize = 4096
            };
        }

        // 初始化绘图
        private void InitializePlot()
        {
            // 为每个需要绘图的参数创建一个PlotModel
            foreach (var param in plotParameters)
            {
                var plotModel = new PlotModel { Title = null }; // 移除标题
                var linearAxisY = new LinearAxis
                {
                    Position = AxisPosition.Left,
                    Title = param, // 设置Y轴标签为参数名称
                    TitleFont = "Arial",
                    TitleFontSize = 12
                };
                var linearAxisX = new LinearAxis
                {
                    Position = AxisPosition.Bottom,
                    IsZoomEnabled = true, // 启用X轴自动缩放
                    IsPanEnabled = true
                };
                var series = new LineSeries { Title = param };
                series.InterpolationAlgorithm = InterpolationAlgorithms.CanonicalSpline; // 设置平滑曲线
                lineSeries[param] = series;
                plotModel.Series.Add(series);
                plotModel.Axes.Add(linearAxisY); // 添加Y轴
                plotModel.Axes.Add(linearAxisX); // 添加X轴
                plotModels[param] = plotModel;

                // 初始化100Hz缓冲区
                buffer100HzByParam[param] = new List<double>();
            }

            // 将PlotModel绑定到对应的PlotView
            a_plot.Model = plotModels["a"];
            q_plot.Model = plotModels["q"];
            delta_plot.Model = plotModels["delta"];
            f_plot.Model = plotModels["f"];
            theta_plot.Model = plotModels["theta"];
            Omega_plot.Model = plotModels["Omega"];
        }

        // 初始化参数与控件映射
        private void InitializeTextBoxMap()
        {
            // 将参数与对应的TextBox控件关联
            textBoxMap["a"] = a;
            textBoxMap["q"] = q;
            textBoxMap["delta"] = delta;
            textBoxMap["f"] = f;
            textBoxMap["Temp"] = Temp;
            textBoxMap["a_PID"] = a_PID;
            textBoxMap["q_PID"] = q_PID;
            textBoxMap["theta"] = theta;
            textBoxMap["Omega"] = Omega;
            textBoxMap["theta_PID"] = theta_PID;
            textBoxMap["cx"] = cx;
            textBoxMap["cy"] = cy;
            textBoxMap["sx"] = sx;
            textBoxMap["sy"] = sy;
            textBoxMap["scale"] = scale;
        }

        // 初始化格式化规则
        private void InitializeFormatRules()
        {
            // 定义每个参数的格式化规则
            formatRules["Temp"] = "F3"; // 保留三位小数
            formatRules["cx"] = "F0"; // 取整数
            formatRules["cy"] = "F0";
            formatRules["sx"] = "F0";
            formatRules["sy"] = "F0";
            formatRules["a"] = "F6"; // 保留六位小数
            formatRules["q"] = "F6";
            formatRules["theta"] = "F6";
            formatRules["delta"] = "F6";
            formatRules["f"] = "F6";
            formatRules["Omega"] = "F6";
            formatRules["a_PID"] = "F6";
            formatRules["q_PID"] = "F6";
            formatRules["theta_PID"] = "F6";
            formatRules["scale"] = "F6";
        }

        // 窗口关闭事件
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            CloseSerialPort(); // 关闭串口
            cancellationTokenSource?.Cancel(); // 取消线程
            CloseFileWriters(); // 关闭文件写入流
            isProcessing = false; // 停止处理线程
            isReceiving = false; // 停止接收线程
            if (processingThread != null && processingThread.IsAlive)
                processingThread.Join(); // 等待处理线程结束
            if (receiveThread != null && receiveThread.IsAlive)
                receiveThread.Join(); // 等待接收线程结束
        }

        // 关闭串口
        private void CloseSerialPort()
        {
            if (serialPort != null && serialPort.IsOpen)
                serialPort.Close();
        }

        // 关闭文件写入流
        private void CloseFileWriters()
        {
            writer1Hz?.Close();
            writer100Hz?.Close();
        }

        // 启动接收线程
        private void StartReceiveThread()
        {
            receiveThread = new Thread(ReceiveData)
            {
                IsBackground = true
            };
            receiveThread.Start();
        }

        // 启动处理线程
        private void StartProcessingThread()
        {
            processingThread = new Thread(ProcessData)
            {
                IsBackground = true
            };
            processingThread.Start();
            isProcessing = true;
        }

        // 接收数据线程
        private void ReceiveData()
        {
            isReceiving = true;
            while (isReceiving)
            {
                if (serialPort != null && serialPort.IsOpen)
                {
                    try
                    {
                        int bytesToRead = serialPort.BytesToRead;
                        if (bytesToRead > 0)
                        {
                            serialPort.Read(receiveBuffer, 0, bytesToRead);
                            lock (queueLock)
                            {
                                byte[] bufferCopy = new byte[bytesToRead];
                                Array.Copy(receiveBuffer, bufferCopy, bytesToRead);
                                dataQueue.Enqueue(bufferCopy); // 将接收到的数据加入队列
                            }
                        }
                    }
                    catch
                    {
                        // 忽略异常
                    }
                }
                Thread.Sleep(1);
            }
        }

        // 处理数据线程
        private void ProcessData()
        {
            while (isProcessing)
            {
                byte[] data;
                lock (queueLock)
                {
                    if (dataQueue.Count > 0)
                    {
                        data = dataQueue.Dequeue(); // 从队列中取出数据
                    }
                    else
                    {
                        data = null;
                    }
                }

                if (data != null)
                {
                    foreach (byte b in data)
                    {
                        ProcessDataByte(b); // 处理每个字节
                    }
                }

                Thread.Sleep(1);
            }
        }

        // 处理单个字节数据
        private void ProcessDataByte(byte dataByte)
        {
            // 将字节添加到缓冲区
            fifoBuffer.Append((char)dataByte);

            // 检查是否有一个完整数据包（以换行符结束）
            int newlineIndex = fifoBuffer.ToString().IndexOf('\n');
            while (newlineIndex != -1)
            {
                string dataStr = fifoBuffer.ToString().Substring(0, newlineIndex).Trim();
                fifoBuffer.Remove(0, newlineIndex + 1);

                if (!string.IsNullOrEmpty(dataStr))
                {
                    string[] values = dataStr.Split(',');
                    if (values.Length == 15)
                    {
                        Dictionary<string, double> parameters = new Dictionary<string, double>();
                        int index = 0;
                        foreach (var key in textBoxMap.Keys)
                        {
                            if (double.TryParse(values[index], NumberStyles.Any, CultureInfo.InvariantCulture, out double value))
                            {
                                parameters[key] = value;
                                index++;
                            }
                        }

                        HandleReceivedParameters(parameters); // 处理接收到的参数
                    }
                }

                // 继续查找下一个换行符
                newlineIndex = fifoBuffer.ToString().IndexOf('\n');
            }
        }

        // 处理接收到的参数
        private void HandleReceivedParameters(Dictionary<string, double> parameters)
        {
            Counter_100Hz++; // 增加100Hz计数器

            // 构建数据行
            string timestamp = DateTime.Now.ToString("HH:mm:ss");
            string dataLine = $"{timestamp} {Counter_100Hz} {string.Join(" ", parameters.Values)}";

            // 保存100Hz数据
            SaveDataToFile(dataLine, filePath100Hz, isSave100HzEnabled, writer100Hz);

            // 将数据添加到100Hz缓冲区
            foreach (var param in parameters)
            {
                if (plotParameters.Contains(param.Key))
                {
                    buffer100HzByParam[param.Key].Add(param.Value);
                }
            }

            // 将数据添加到所有参数缓冲区
            foreach (var param in parameters)
            {
                if (!buffer1HzByParams.ContainsKey(param.Key))
                {
                    buffer1HzByParams[param.Key] = new List<double>();
                }
                buffer1HzByParams[param.Key].Add(param.Value);
            }

            // 检查是否需要更新UI
            if ((DateTime.Now - lastUIUpdate).TotalMilliseconds >= UI_UPDATE_INTERVAL)
            {
                UpdateUI(); // 更新UI
                lastUIUpdate = DateTime.Now;
            }

            // 处理1Hz数据存储
            Handle1HzStorage(parameters);
        }

        // 更新UI
        private void UpdateUI()
        {
            Application.Current.Dispatcher.Invoke(() =>
            {
                // 更新所有参数的TextBox
                foreach (var param in buffer1HzByParams.Keys)
                {
                    if (buffer1HzByParams[param].Count > 0)
                    {
                        double latestValue = buffer1HzByParams[param][buffer1HzByParams[param].Count - 1];
                        if (textBoxMap.TryGetValue(param, out TextBox textBox))
                        {
                            if (formatRules.TryGetValue(param, out string format))
                            {
                                textBox.Text = latestValue.ToString(format, CultureInfo.InvariantCulture);
                            }
                            else
                            {
                                textBox.Text = latestValue.ToString();
                            }
                        }
                    }
                }

                // 更新100Hz绘图
                if (isDraw100HzSelected)
                {
                    foreach (var param in plotParameters)
                    {
                        if (buffer100HzByParam[param].Count > 0)
                        {
                            // 使用Counter_100Hz作为x轴的基准值
                            for (int i = 0; i < buffer100HzByParam[param].Count; i++)
                            {
                                lineSeries[param].Points.Add(new DataPoint((Counter_100Hz - buffer100HzByParam[param].Count + i) / 100.0, buffer100HzByParam[param][i]));
                            }
                            plotModels[param].InvalidatePlot(true);
                            buffer100HzByParam[param].Clear();
                        }
                    }
                }
                else if (isDraw1HzSelected)
                {
                    // 更新1Hz绘图
                    foreach (var param in plotParameters)
                    {
                        if (buffer100HzByParam[param].Count > 0)
                        {
                            double latestValue = buffer100HzByParam[param][buffer100HzByParam[param].Count - 1];
                            lineSeries[param].Points.Add(new DataPoint(Counter_1Hz, latestValue));
                            plotModels[param].InvalidatePlot(true);
                        }
                    }
                }

                // 更新计数器显示
                current_time.Text = Counter_1Hz.ToString();

                // 检查是否达到采样时间
                if (int.TryParse(sample_time.Text, out sampleTime))
                {
                    TimeSpan elapsedTime = DateTime.Now - startTime;
                    if (elapsedTime.TotalSeconds >= sampleTime)
                    {
                        CloseSerialPort();
                        UpdateStartButtonState(false);
                        EnableControlsOnSerialPortClose();
                        CloseFileWriters();
                        cancellationTokenSource?.Cancel();
                        MessageBox.Show("采样时间已到，停止采集！", "提示", MessageBoxButton.OK, MessageBoxImage.Information);
                    }
                }
                else
                {
                    MessageBox.Show("请输入有效的采样时间（秒）", "提示", MessageBoxButton.OK, MessageBoxImage.Warning);
                }
            });
        }

        // 处理1Hz数据存储
        private void Handle1HzStorage(Dictionary<string, double> parameters)
        {
            // 检查是否累积了100个数据点
            bool allParamsHave100Points = true;
            foreach (var param in parameters.Keys)
            {
                if (!buffer1HzByParams.ContainsKey(param) || buffer1HzByParams[param].Count < 100)
                {
                    allParamsHave100Points = false;
                    break;
                }
            }

            if (allParamsHave100Points)
            {
                Dictionary<string, double> averagedParameters = new Dictionary<string, double>();
                foreach (var param in parameters.Keys)
                {
                    averagedParameters[param] = buffer1HzByParams[param].Average();
                    buffer1HzByParams[param].Clear(); // 清空缓冲区
                }

                Counter_1Hz++;
                string timestamp = DateTime.Now.ToString("HH:mm:ss");
                string dataLine = $"{timestamp} {Counter_1Hz * 100} {string.Join(" ", averagedParameters.Values)}";

                // 保存1Hz数据
                Save1HzDataToFile(dataLine);

                // 更新1Hz绘图
                Application.Current.Dispatcher.Invoke(() =>
                {
                    if (isDraw1HzSelected)
                    {
                        foreach (var param in averagedParameters)
                        {
                            if (plotParameters.Contains(param.Key))
                            {
                                lineSeries[param.Key].Points.Add(new DataPoint(Counter_1Hz, param.Value));
                                plotModels[param.Key].InvalidatePlot(true);
                            }
                        }
                    }
                });
            }
        }

        // 保存数据到文件
        private void SaveDataToFile(string dataLine, string filePath, bool isEnabled, StreamWriter writer)
        {
            if (isEnabled && writer != null)
            {
                try
                {
                    writer.WriteLine(dataLine);
                    writer.Flush();
                }
                catch (Exception ex)
                {
                    ShowSaveFailedMessage(ex.Message);
                }
            }
        }

        // 保存1Hz数据到文件
        private void Save1HzDataToFile(string dataLine)
        {
            if (isSave1HzEnabled && writer1Hz != null)
            {
                try
                {
                    // 解析原始数据行
                    string[] parts = dataLine.Split(' ');
                    string timestamp = parts[0];
                    string counter = parts[1];
                    string[] values = parts.Skip(2).ToArray();

                    // 应用格式化规则
                    List<string> formattedValues = new List<string>();
                    int index = 0;
                    foreach (var param in buffer1HzByParams.Keys)
                    {
                        if (formatRules.TryGetValue(param, out string format))
                        {
                            double value = double.Parse(values[index]);
                            formattedValues.Add(value.ToString(format, CultureInfo.InvariantCulture));
                            index++;
                        }
                        else
                        {
                            formattedValues.Add(values[index]);
                            index++;
                        }
                    }

                    // 构建新的数据行
                    string formattedDataLine = $"{timestamp} {counter} {string.Join(" ", formattedValues)}";

                    // 写入文件
                    if (linesWritten1Hz == 0)
                    {
                        writer1Hz.WriteLine(formattedDataLine);
                        linesWritten1Hz++;
                    }
                    else
                    {
                        writer1Hz.WriteLine(formattedDataLine);
                        writer1Hz.Flush();
                    }
                }
                catch (Exception ex)
                {
                    ShowSaveFailedMessage($"1Hz保存失败: {ex.Message}");
                }
            }
        }

        // 显示保存失败消息
        private void ShowSaveFailedMessage(string message)
        {
            if (Application.Current != null && Application.Current.Dispatcher != null)
            {
                if (Application.Current.Dispatcher.CheckAccess())
                {
                    MessageBox.Show($"保存失败: {message}");
                }
                else
                {
                    Application.Current.Dispatcher.Invoke(() =>
                    {
                        MessageBox.Show($"保存失败: {message}");
                    });
                }
            }
            else
            {
                MessageBox.Show($"保存失败: {message}");
            }
        }

        // 开始/停止按钮点击事件
        private void start_Click(object sender, RoutedEventArgs e)
        {
            if (!serialPort.IsOpen)
            {
                if (!isDraw100HzSelected && !isDraw1HzSelected)
                {
                    MessageBox.Show("请先选择绘图频率（1Hz或100Hz）",
                        "提示",
                        MessageBoxButton.OK,
                        MessageBoxImage.Warning);
                    return;
                }
                try
                {
                    ResetCountersAndPlot(); // 重置计数器和绘图
                    CloseFileWriters(); // 关闭文件写入流
                    InitializeFileWriters(); // 初始化文件写入流
                    ConfigureAndOpenSerialPort(); // 配置并打开串口
                    UpdateStartButtonState(true); // 更新按钮状态

                    // 记录开始时间
                    startTime = DateTime.Now;
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message);
                }
            }
            else
            {
                CloseSerialPort(); // 关闭串口
                UpdateStartButtonState(false); // 更新按钮状态
                EnableControlsOnSerialPortClose(); // 启用控件
                CloseFileWriters(); // 关闭文件写入流
                cancellationTokenSource?.Cancel(); // 取消线程
            }
        }

        // 重置计数器和绘图
        private void ResetCountersAndPlot()
        {
            Counter_100Hz = 0;
            linesWritten1Hz = 0;
            Counter_1Hz = 0;

            // 清空绘图数据
            foreach (var param in plotParameters)
            {
                lineSeries[param].Points.Clear();
                plotModels[param].InvalidatePlot(true);
            }

            // 清空缓冲区
            foreach (var param in plotParameters)
            {
                buffer100HzByParam[param].Clear();
            }
            foreach (var param in buffer1HzByParams.Keys)
            {
                buffer1HzByParams[param].Clear();
            }
        }

        // 初始化文件写入流
        private void InitializeFileWriters()
        {
            if (isSave1HzEnabled && !string.IsNullOrEmpty(filePath1Hz))
            {
                File.WriteAllText(filePath1Hz, string.Empty);
                writer1Hz = new StreamWriter(filePath1Hz, true);
            }

            if (isSave100HzEnabled && !string.IsNullOrEmpty(filePath100Hz))
            {
                File.WriteAllText(filePath100Hz, string.Empty);
                writer100Hz = new StreamWriter(filePath100Hz, true);
            }
        }

        // 配置并打开串口
        private void ConfigureAndOpenSerialPort()
        {
            serialPort.PortName = com.SelectedItem.ToString();
            serialPort.BaudRate = int.Parse(Baud_rate.SelectedItem.ToString());
            serialPort.Open();
        }

        // 更新开始按钮状态
        private void UpdateStartButtonState(bool isOpen)
        {
            start.Content = isOpen ? "关闭" : "开始";
            Baud_rate.IsEnabled = !isOpen;
            com.IsEnabled = !isOpen;
            bool areControlsDisabled = isOpen;
            save_1Hz.IsEnabled = !areControlsDisabled;
            save_100Hz.IsEnabled = !areControlsDisabled;
            draw_100Hz.IsEnabled = !areControlsDisabled;
            draw_1Hz.IsEnabled = !areControlsDisabled;
        }

        // 在串口关闭时启用控件
        private void EnableControlsOnSerialPortClose()
        {
            Baud_rate.IsEnabled = true;
            com.IsEnabled = true;
            save_1Hz.IsEnabled = true;
            save_100Hz.IsEnabled = true;
            draw_100Hz.IsEnabled = true;
            draw_1Hz.IsEnabled = true;
        }

        // 100Hz绘图选择事件
        private void draw_100Hz_Checked(object sender, RoutedEventArgs e)
        {
            isDraw100HzSelected = true;
            isDraw1HzSelected = false;
        }

        // 1Hz绘图选择事件
        private void draw_1Hz_Checked(object sender, RoutedEventArgs e)
        {
            isDraw1HzSelected = true;
            isDraw100HzSelected = false;
        }

        // 1Hz保存选择事件
        private void save_1Hz_Checked(object sender, RoutedEventArgs e)
        {
            btn_1Hz.IsEnabled = true;
            save_path_1Hz.IsEnabled = true;
            isSave1HzEnabled = true;
        }

        // 1Hz保存取消事件
        private void save_1Hz_Unchecked(object sender, RoutedEventArgs e)
        {
            DisableSaveControls(save_1Hz, btn_1Hz, save_path_1Hz);
            filePath1Hz = string.Empty;
            isSave1HzEnabled = false;
        }

        // 100Hz保存选择事件
        private void save_100Hz_Checked(object sender, RoutedEventArgs e)
        {
            btn_100Hz.IsEnabled = true;
            save_path_100Hz.IsEnabled = true;
            isSave100HzEnabled = true;
        }

        // 100Hz保存取消事件
        private void save_100Hz_Unchecked(object sender, RoutedEventArgs e)
        {
            DisableSaveControls(save_100Hz, btn_100Hz, save_path_100Hz);
            filePath100Hz = string.Empty;
            isSave100HzEnabled = false;
        }

        // 禁用保存控件
        private void DisableSaveControls(CheckBox saveCheckBox, Button button, TextBox textBox)
        {
            button.IsEnabled = false;
            textBox.IsEnabled = false;
            saveCheckBox.IsChecked = false;
        }

        // 选择1Hz数据保存路径
        private void btn_1Hz_Click(object sender, RoutedEventArgs e)
        {
            SelectSaveFilePath(ref filePath1Hz, save_path_1Hz, "1Hz_数据");
        }

        // 选择100Hz数据保存路径
        private void btn_100Hz_Click(object sender, RoutedEventArgs e)
        {
            SelectSaveFilePath(ref filePath100Hz, save_path_100Hz, "100Hz_数据");
        }

        // 选择保存文件路径
        private void SelectSaveFilePath(ref string filePath, TextBox textBox, string defaultFileName)
        {
            SaveFileDialog saveFileDialog = new SaveFileDialog
            {
                Filter = "Text files (*.txt)|*.txt|All files (*.*)|*.*",
                FileName = defaultFileName,
                DefaultExt = "txt"
            };
            if (saveFileDialog.ShowDialog() == true)
            {
                filePath = saveFileDialog.FileName;
                textBox.Text = filePath;
            }
        }
    }
}