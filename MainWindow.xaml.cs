using HMI_EC;
using Microsoft.Win32;
using ScottPlot;
using ScottPlot.WPF;
using SkiaSharp;
using System.ComponentModel;
using System.Globalization;
using System.IO;
using System.IO.Ports;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;

namespace HMI
{
    public partial class MainWindow : Window
    {
        // 声明误差补偿窗口的实例
        private ECWindow _ecWindow;
        // 变量初始化
        // ======== 串口通信相关 ========
        private SerialPort serialPort; // 串口对象
        private Queue<byte[]> dataQueue = new(); // 接收到的数据队列
        private object queueLock = new(); // 用于线程安全锁对象
        private byte[] receiveBuffer = new byte[8192]; // 接收缓冲区
        private volatile bool isReceiving = false; // 标志是否正在接收数据
        private volatile bool isProcessing = false; // 标志是否正在处理数据
        private Thread receiveThread; // 接收数据的线程
        private Thread processingThread; // 处理数据的线程

        // ======== 数据存储与保存相关 ========
        private volatile bool isSave1HzEnabled = false; // 是否启用1Hz数据保存
        private volatile bool isSave100HzEnabled = false; // 是否启用100Hz数据保存
        private string filePath1Hz = string.Empty; // 1Hz数据保存路径
        private string filePath100Hz = string.Empty; // 100Hz数据保存路径
        private StreamWriter writer1Hz = null; // 1Hz数据写入流
        private StreamWriter writer100Hz = null; // 100Hz数据写入流
        private bool isStoppingDueToSampleTime = false; // 标志是否因为到达采样时间而停止
        private CancellationTokenSource cancellationTokenSource; // 用于取消任务的令牌源

        // ======== 数据处理与绘图相关 ========
        private Dictionary<string, (double[] ys, WpfPlot plot)> signalPlots = new(); // 存储绘图数据和控件
        private Dictionary<string, List<double>> buffer100HzByParams = new(); // 100Hz数据缓冲区
        private Dictionary<string, List<double>> buffer1HzByParams = new(); // 1Hz数据缓冲区
        private Dictionary<string, List<double>> bufferplot100HzByParams = new(); // 绘图数据缓冲区
        private Dictionary<string, List<double>> bufferplot1HzByParams = new(); // 1Hz绘图缓存区
        private volatile int Counter_100Hz = 0; // 100Hz数据计数器
        private volatile int Counter_1Hz = 0; // 1Hz数据计数器
        private DateTime lastUIUpdate = DateTime.Now; // 上次UI更新时间
        private const int UI_UPDATE_INTERVAL = 1000; // UI更新间隔（毫秒）
        private bool isDraw100HzSelected = false; // 是否选择100Hz绘图
        private bool isDraw1HzSelected = false; // 是否选择1Hz绘图
        private bool isAutoFitEnabled = false; // 是否启用自动缩放

        // ======== 参数与控件映射相关 ========
        private Dictionary<string, TextBox> textBoxMap = new(); // 参数与文本框的映射
        private Dictionary<string, WpfPlot> plotParameterToControlMap = new(); // 参数与绘图控件的映射
        private Dictionary<string, string> formatRules = new(); // 参数格式化规则
        private StringBuilder fifoBuffer = new StringBuilder(); // 用于解析数据的缓冲区
        private int sampleTime = 3600; // 采样时间（秒）
        private DateTime startTime; // 开始时间

        // ======== 构造函数 ========
        public MainWindow()
        {
            InitializeComponent(); // 初始化组件
            this.Closing += MainWindow_Closing; // 注册窗口关闭事件

            InitializeControls(); // 初始化控件
            InitializeSerialPort(); // 初始化串口
            InitializePlot(); // 初始化绘图
            InitializeTextBoxMap(); // 初始化文本框映射
            InitializeFormatRules(); // 初始化格式化规则

            StartReceiveThread(); // 启动接收线程
            StartProcessingThread(); // 启动处理线程
        }

        //********初始化功能模块********
        // ======== 初始化控件 ========
        private void InitializeControls()
        {
            // 初始化串口名称下拉框
            string[] portNames = SerialPort.GetPortNames();
            com.ItemsSource = portNames;
            com.SelectedIndex = 0;

            // 初始化波特率下拉框
            string[] baudRates = { "9600", "19200", "38400", "57600", "115200", "460800" };
            Baud_rate.ItemsSource = baudRates;
            Baud_rate.SelectedIndex = 5;

            // 初始化采样时间文本框
            sample_time.Text = "3600";

            // 初始化Auto状态
            Cb_AutoFit.IsChecked = false; // 默认未选中
        }

        // ======== 初始化串口 ========
        private void InitializeSerialPort()
        {
            serialPort = new SerialPort
            {
                Parity = Parity.None, // 无校验位
                StopBits = StopBits.One, // 1个停止位
                DataBits = 8, // 8个数据位
                Handshake = Handshake.None, // 无握手协议
                ReadBufferSize = 4096 // 读取缓冲区大小
            };
        }

        // ======== 启动接收线程 ========
        private void StartReceiveThread()
        {
            receiveThread = new Thread(ReceiveData)
            {
                IsBackground = true // 设置为后台线程
            };
            receiveThread.Start(); // 启动线程
        }

        // ======== 启动处理线程 ========
        private void StartProcessingThread()
        {
            processingThread = new Thread(ProcessData)
            {
                IsBackground = true // 设置为后台线程
            };
            processingThread.Start(); // 启动线程
            isProcessing = true; // 标志数据处理已启动
        }

        // ======== 初始化绘图 ========
        private void InitializePlot()
        {
            // 初始化参数与绘图控件的映射
            plotParameterToControlMap = new Dictionary<string, WpfPlot>
            {
                { "a", a_plot },
                { "q", q_plot },
                { "theta", theta_plot },
                { "delta", delta_plot },
                { "f", f_plot },
                { "Omega", Omega_plot }
            };

            // 初始化每个参数的绘图控件
            foreach (var param in plotParameterToControlMap.Keys)
            {
                var plot = plotParameterToControlMap[param]; // 获取绘图控件
                PixelPadding padding = new(71, 4, 19, 7);
                plot.Plot.Layout.Fixed(padding);
                plot.Plot.ScaleFactor = 0.8;
                double[] ys = new double[0]; // 初始化Y轴数据
                plot.Plot.Axes.SetLimitsX(0, 3600); // 设置X轴范围
                signalPlots[param] = (ys, plot); // 存储绘图数据和控件
                plot.Plot.YLabel(param); // 设置Y轴标签
                plot.Refresh(); // 刷新绘图

                // 设置Y轴刻度生成器为自定义的DynamicPrecisionFixedTicksGenerator
                DynamicPrecisionFixedTicksGenerator tickGenerator = new();
                plot.Plot.Axes.Left.TickGenerator = tickGenerator;

                // 初始化绘图数据缓冲区
                bufferplot100HzByParams[param] = new();
                bufferplot1HzByParams[param] = new();
                // 设置Y轴精度到小数点后一位
                //var tickGenerator = new ScottPlot.TickGenerators.NumericAutomatic();
                //tickGenerator.LabelFormatter = new Func<double, string>((value) => value.ToString("0.000", CultureInfo.InvariantCulture));
                //plot.Plot.Axes.Left.TickGenerator = tickGenerator;
            }
            // 关联所有X轴
            LinkAllXAxes();
        }
        public class DynamicPrecisionFixedTicksGenerator : ITickGenerator
        {
            private int minTickCount = 3; // 最小刻度数量
            public int MaxTickCount { get; set; } = 5; // 最大刻度数量

            public Tick[] Ticks { get; private set; } = Array.Empty<Tick>();

            public void Regenerate(CoordinateRange range, Edge edge, PixelLength length, SKPaint paint, LabelStyle labelStyle)
            {
                // 根据数据范围和绘图区域长度计算最优刻度数量
                int optimalTickCount = GetOptimalTickCount(range, length);

                // 确保刻度数量在 minTickCount 和 MaxTickCount 之间
                int tickCount = Math.Clamp(optimalTickCount, minTickCount, MaxTickCount);

                // 计算步长
                double step = (range.Max - range.Min) / (tickCount - 1);

                // 生成刻度
                Ticks = new Tick[tickCount];
                for (int i = 0; i < tickCount; i++)
                {
                    double value = range.Min + i * step;
                    string format = GetFormatForRange(range);
                    Ticks[i] = new Tick(value, value.ToString(format, CultureInfo.InvariantCulture));
                }
            }

            private int GetOptimalTickCount(CoordinateRange range, PixelLength length)
            {
                // 根据数据范围和绘图区域长度计算最优刻度数量
                double rangeValue = Math.Abs(range.Max - range.Min);
                double pixelPerUnit = length.Length / rangeValue;
                return (int)(pixelPerUnit / 50);
            }

            private string GetFormatForRange(CoordinateRange range)
            {
                double rangeValue = Math.Abs(range.Max - range.Min);
                int decimalPlaces = rangeValue switch
                {
                    < 0.00001 => 6,
                    < 0.0001 => 5,
                    < 0.001 => 4,
                    < 0.01 => 3,
                    < 0.1 => 2,
                    _ => 1
                };
                return $"0.{new string('0', decimalPlaces)}";
            }
        }

        // ======== 初始化参数与控件映射 ========
        private void InitializeTextBoxMap()
        {
            // 将参数与文本框控件进行映射
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

        // ======== 初始化格式化规则 ========
        private void InitializeFormatRules()
        {
            // 定义参数的格式化规则
            formatRules["Temp"] = "F3";
            formatRules["cx"] = "F0";
            formatRules["cy"] = "F0";
            formatRules["sx"] = "F0";
            formatRules["sy"] = "F0";
            formatRules["a"] = "F6";
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

        //********数据接收与处理模块********
        // ======== 接收数据线程 ========
        private void ReceiveData()
        {
            isReceiving = true; // 标志数据接收已启动
            while (isReceiving)
            {
                if (serialPort != null && serialPort.IsOpen)
                {
                    int bytesToRead = serialPort.BytesToRead; // 获取可读取的字节数
                    if (bytesToRead > 0)
                    {
                        // 从串口读取数据
                        serialPort.Read(receiveBuffer, 0, bytesToRead);

                        // 将数据复制到队列中
                        lock (queueLock)
                        {
                            byte[] bufferCopy = new byte[bytesToRead];
                            Array.Copy(receiveBuffer, bufferCopy, bytesToRead);
                            dataQueue.Enqueue(bufferCopy);
                        }
                    }
                }
                Thread.Sleep(10);
            }
        }

        // ======== 处理数据线程 ========
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
                    // 处理每个字节
                    foreach (byte b in data)
                    {
                        ProcessDataByte(b);
                    }
                }

                Thread.Sleep(10);
            }
        }

        // ======== 处理单个字节数据 ========
        private void ProcessDataByte(byte dataByte)
        {
            fifoBuffer.Append((char)dataByte); // 将字节转换为字符并添加到缓冲区

            // 查找换行符
            int newlineIndex = fifoBuffer.ToString().IndexOf('\n');
            while (newlineIndex != -1)
            {
                // 提取一行数据
                string dataStr = fifoBuffer.ToString().Substring(0, newlineIndex).Trim();
                fifoBuffer.Remove(0, newlineIndex + 1);

                if (!string.IsNullOrEmpty(dataStr))
                {
                    // 分割数据
                    string[] values = dataStr.Split(',');
                    if (values.Length == 15)
                    {
                        Dictionary<string, double> parameters = new();
                        int index = 0;

                        // 解析参数
                        foreach (var key in textBoxMap.Keys)
                        {
                            if (double.TryParse(values[index], NumberStyles.Any, CultureInfo.InvariantCulture, out double value))
                            {
                                parameters[key] = value;
                                index++;
                            }
                        }

                        // 处理接收到的参数
                        HandleReceivedParameters(parameters);
                    }
                }

                // 继续查找换行符
                newlineIndex = fifoBuffer.ToString().IndexOf('\n');
            }
        }

        // ======== 处理接收到的参数 ========
        private void HandleReceivedParameters(Dictionary<string, double> parameters)
        {
            // 构建数据行
            string timestamp = DateTime.Now.ToString("HH:mm:ss");
            string dataLine = $"{timestamp}\t{Counter_100Hz + 1}\t{string.Join("\t", parameters.Values)}";

            // 保存100Hz数据
            Save100HzDataToFile(dataLine);

            // 更新100Hz绘图数据缓冲区
            foreach (var param in parameters)
            {
                if (plotParameterToControlMap.ContainsKey(param.Key))
                {
                    bufferplot100HzByParams[param.Key].Add(param.Value);
                }
            }

            // 更新1Hz绘图缓存区
            foreach (var param in parameters)
            {
                if (!bufferplot1HzByParams.ContainsKey(param.Key))
                {
                    bufferplot1HzByParams[param.Key] = new();
                }
                bufferplot1HzByParams[param.Key].Add(param.Value);
            }

            // 更新100Hz数据缓冲区
            foreach (var param in parameters)
            {
                if (!buffer100HzByParams.ContainsKey(param.Key))
                {
                    buffer100HzByParams[param.Key] = new();
                }
                buffer100HzByParams[param.Key].Add(param.Value);
            }

            // 更新1Hz数据缓冲区
            foreach (var param in parameters)
            {
                if (!buffer1HzByParams.ContainsKey(param.Key))
                {
                    buffer1HzByParams[param.Key] = new();
                }
                buffer1HzByParams[param.Key].Add(param.Value);
            }

            // 检查是否需要更新UI
            if ((DateTime.Now - lastUIUpdate).TotalMilliseconds >= UI_UPDATE_INTERVAL)
            {
                UpdateUI(); // 更新UI
                lastUIUpdate = DateTime.Now; // 更新上次UI更新时间
            }

            // 处理1Hz数据存储
            Handle1HzStorage(parameters);
        }

        // ======== 处理1Hz数据存储 ========
        private void Handle1HzStorage(Dictionary<string, double> parameters)
        {
            // 检查是否所有参数都有足够的数据点
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
                // 计算每个参数的平均值
                Dictionary<string, double> averagedParameters = new();
                foreach (var param in parameters.Keys)
                {
                    averagedParameters[param] = buffer1HzByParams[param].Average();
                    buffer1HzByParams[param].Clear(); // 清空缓冲区
                }

                Counter_1Hz++; // 增加1Hz计数器
                string timestamp = DateTime.Now.ToString("HH:mm:ss");
                string dataLine = $"{timestamp}\t{Counter_1Hz * 100}\t{string.Join("\t", averagedParameters.Values)}";

                // 保存1Hz数据
                Save1HzDataToFile(dataLine);
            }
        }

        //********文件保存模块********
        // ======== 保存数据到文件 ========
        private void Save100HzDataToFile(string dataLine)
        {
            if (isSave100HzEnabled && writer100Hz != null)
            {
                try
                {
                    // 解析数据行
                    string[] parts = dataLine.Split('\t');
                    string timestamp = parts[0];
                    string counter = parts[1];
                    string[] values = parts.Skip(2).ToArray();

                    // 检查values的长度是否与buffer100HzByParams的键值对数量一致
                    if (values.Length == buffer100HzByParams.Count)
                    {
                        // 格式化数据
                        List<string> formattedValues = new();
                        int index = 0;
                        foreach (var param in buffer100HzByParams.Keys)
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

                        string formattedDataLine = $"{timestamp}\t{counter}\t{string.Join("\t", formattedValues)}";

                        // 写入数据
                        writer100Hz.WriteLine(formattedDataLine);
                        writer100Hz.Flush();
                        // 只有在数据格式正确时才递增计数器
                        Counter_100Hz++;
                    }
                    else
                    {
                        // 如果不一致，记录警告日志
                        //UpdateState("警告: 数据格式与参数数量不匹配，可能丢失数据。\n");
                        return; // 不递增计数器
                    }
                }
                catch
                {
                    // 检查是否因为任务取消导致的异常
                    //if (!isStoppingDueToSampleTime)
                    //{
                        //UpdateState("100Hz采集结束");
                    //}
                }
            }
        }

        // ======== 保存1Hz数据到文件 ========
        private void Save1HzDataToFile(string dataLine)
        {
            try
            {
                if (isSave1HzEnabled && writer1Hz != null)
                {
                    string[] parts = dataLine.Split('\t');
                    string timestamp = parts[0];
                    string counter = parts[1];
                    string[] values = parts.Skip(2).ToArray();

                    // 格式化数据
                    List<string> formattedValues = new();
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

                    string formattedDataLine = $"{timestamp}\t{counter}\t{string.Join("\t", formattedValues)}";

                    writer1Hz.WriteLine(formattedDataLine);
                    writer1Hz.Flush();
                }
            }
            catch
            {
                // 检查是否因为任务取消导致的异常
                //if (!isStoppingDueToSampleTime)
                //{
                //    UpdateState("1Hz采集结束");
                //}
            }
        }

        // ======== 显示保存失败消息 ======== 
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

        // ======== 初始化文件写入流 ========
        private void InitializeFileWriters()
        {
            if (isSave1HzEnabled && !string.IsNullOrEmpty(filePath1Hz))
            {
                try
                {
                    File.WriteAllText(filePath1Hz, string.Empty); // 清空文件
                    writer1Hz = new StreamWriter(filePath1Hz, true); // 创建写入流
                    UpdateState("\n1Hz 保存地址设置正确！ \n");
                }
                catch (Exception ex)
                {
                    ShowSaveFailedMessage(ex.Message);
                    UpdateState("错误: 1Hz 保存地址无效！！ \n");
                }
            }

            if (isSave100HzEnabled && !string.IsNullOrEmpty(filePath100Hz))
            {
                try
                {
                    File.WriteAllText(filePath100Hz, string.Empty); // 清空文件
                    writer100Hz = new StreamWriter(filePath100Hz, true); // 创建写入流
                    UpdateState("\n100Hz 保存地址设置正确！ \n");
                }
                catch (Exception ex)
                {
                    ShowSaveFailedMessage(ex.Message);
                    UpdateState("错误: 100Hz 保存地址无效！！ \n");
                }
            }

            // 检查1Hz和100Hz的保存地址是否相同
            if (isSave1HzEnabled && isSave100HzEnabled &&
                !string.IsNullOrEmpty(filePath1Hz) && !string.IsNullOrEmpty(filePath100Hz) &&
                filePath1Hz == filePath100Hz)
            {
                UpdateState("错误: 1Hz 与 100Hz 保存地址相同！ \n");
                return;
            }
        }

        // ======== 选择保存文件路径 ========
        private void SelectSaveFilePath(ref string filePath, TextBox textBox, string defaultFileName, string saveType)
        {
            SaveFileDialog saveFileDialog = new SaveFileDialog
            {
                Filter = "Text files (*.txt)|*.txt|All files (*.*)|*.*", // 文件过滤器
                FileName = defaultFileName, // 默认文件名
                DefaultExt = "txt" // 默认扩展名
            };
            if (saveFileDialog.ShowDialog() == true)
            {
                string fileExtension = Path.GetExtension(saveFileDialog.FileName);
                if (fileExtension.ToLower() != ".txt")
                {
                    if (saveType == "1Hz")
                    {
                        UpdateState("错误: 1Hz 保存地址无效！！ \n");
                    }
                    else
                    {
                        UpdateState("错误: 100Hz 保存地址无效！！ \n");
                    }
                    ShowSaveFailedMessage("无效的地址!");
                    return;
                }
                filePath = saveFileDialog.FileName; // 获取选择的文件路径
                textBox.Text = filePath; // 更新文本框显示
                UpdateState($"设置{defaultFileName}数据保存地址为: {filePath}\n");
            }
        }

        //********UI更新模块********
        // ======== 更新UI ========
        private void UpdateUI()
        {
            Application.Current.Dispatcher.Invoke(() =>
            {
                // 更新文本框显示
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
                    foreach (var param in plotParameterToControlMap.Keys)
                    {
                        if (bufferplot100HzByParams[param].Count >= 100)
                        {
                            var (ys, plot) = signalPlots[param];
                            int newPoints = bufferplot100HzByParams[param].Count;

                            // 更新绘图数据
                            double[] newYs = new double[ys.Length + newPoints];
                            ys.CopyTo(newYs, 0);

                            for (int i = 0; i < newPoints; i++)
                            {
                                newYs[newYs.Length - newPoints + i] = bufferplot100HzByParams[param][i];
                            }

                            // 清空并重新绘制
                            plot.Plot.Clear();
                            plot.Plot.Add.Signal(newYs, 0.01, GetScottPlotColorByParameter(param));
                            if (isAutoFitEnabled)
                            {
                                plot.Plot.Axes.AutoScale();
                            }
                            plot.Refresh();

                            signalPlots[param] = (newYs, plot);
                            bufferplot100HzByParams[param].Clear();
                        }
                    }
                }
                // 更新1Hz绘图
                if (isDraw1HzSelected)
                {
                    foreach (var param in plotParameterToControlMap.Keys)
                    {
                        if (bufferplot1HzByParams[param].Count >= 100)
                        {
                            double averageValue = bufferplot1HzByParams[param].Average(); // 计算平均值
                            bufferplot1HzByParams[param].Clear(); // 清空缓存区
                            var (ys, plot) = signalPlots[param];
                            // 更新绘图数据
                            double[] newYs = new double[ys.Length + 1];
                            ys.CopyTo(newYs, 0);
                            newYs[newYs.Length - 1] = averageValue;

                            // 清空并重新绘制
                            plot.Plot.Clear();
                            plot.Plot.Add.Signal(newYs, 1, GetScottPlotColorByParameter(param));
                            if (isAutoFitEnabled)
                            {
                                plot.Plot.Axes.AutoScale();
                            }
                            plot.Refresh();

                            signalPlots[param] = (newYs, plot);
                        }
                    }
                }

                // 更新当前时间显示
                current_time.Text = Counter_1Hz.ToString();

                // 检查采样时间是否已到
                if (int.TryParse(sample_time.Text, out sampleTime))
                {
                    TimeSpan elapsedTime = DateTime.Now - startTime;
                    if (elapsedTime.TotalSeconds >= sampleTime)
                    {
                        isStoppingDueToSampleTime = true; // 设置标志位
                        cancellationTokenSource?.Cancel(); // 取消任务
                        CloseFileWriters(); // 关闭文件写入流
                        CloseSerialPort(); // 关闭串口
                        UpdateStartButtonState(false); // 更新按钮状态
                        EnableControlsOnSerialPortClose(); // 启用控件
                        MessageBox.Show("数据采集完成！", "提示", MessageBoxButton.OK, MessageBoxImage.Information);
                        UpdateState("数据采集完成！");
                    }
                }
                else
                {
                    MessageBox.Show("错误: 采样时间配置错误", "提示", MessageBoxButton.OK, MessageBoxImage.Warning);
                }
            });
        }

        // ======== 根据参数名称获取颜色 ========
        private ScottPlot.Color GetScottPlotColorByParameter(string param)
        {
            return param switch
            {
                "a" => ScottPlot.Colors.Orange,
                "q" => ScottPlot.Colors.Purple,
                "delta" => ScottPlot.Colors.Blue,
                "f" => ScottPlot.Colors.DeepPink,
                "theta" => ScottPlot.Colors.Green,
                "Omega" => ScottPlot.Colors.Red,
                _ => ScottPlot.Colors.Black
            };
        }

        // ======== 更新状态信息 ======== 
        private void UpdateState(string message)
        {
            Application.Current.Dispatcher.Invoke(() =>
            {
                stateTextBox.AppendText(message + "\n");
                stateTextBox.ScrollToEnd(); // 滚动到底部
            });
        }

        //********按钮事件处理模块********
        // ======== 开始/停止按钮点击事件 ========
        private void Btn_start_Click(object sender, RoutedEventArgs e)
        {
            if (!serialPort.IsOpen)
            {
                // 检查是否选择了绘图频率
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
                    isStoppingDueToSampleTime = false; // 重置标志位
                    ResetCountersAndPlot(); // 重置计数器和绘图
                    CloseFileWriters(); // 关闭文件写入流
                    InitializeFileWriters(); // 初始化文件写入流
                    ConfigureAndOpenSerialPort(); // 配置并打开串口
                    UpdateStartButtonState(true); // 更新按钮状态
                    startTime = DateTime.Now; // 记录开始时间
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message); // 显示异常消息
                }
            }
            else
            {
                CloseSerialPort(); // 关闭串口
                UpdateStartButtonState(false); // 更新按钮状态
                EnableControlsOnSerialPortClose(); // 启用控件
                CloseFileWriters(); // 关闭文件写入流
                cancellationTokenSource?.Cancel(); // 取消任务

                isAutoFitEnabled = false; // 禁用自动缩放
                Cb_AutoFit.IsChecked = false; // 取消选中复选框
            }
        }

        // ======== 重置计数器和绘图 ========
        private void ResetCountersAndPlot()
        {
            Counter_100Hz = 0; // 重置100Hz计数器
            Counter_1Hz = 0; // 重置1Hz计数器

            // 重置绘图
            foreach (var param in plotParameterToControlMap.Keys)
            {
                var (_, plot) = signalPlots[param];
                plot.Plot.Clear(); // 清空绘图
                plot.Plot.Axes.SetLimitsX(0, 3600); // 设置X轴范围
                plot.Refresh(); // 刷新绘图
                signalPlots[param] = (new double[0], plot); // 重置绘图数据
            }

            // 清空缓冲区
            foreach (var param in plotParameterToControlMap.Keys)
            {
                bufferplot100HzByParams[param].Clear();
            }
        }

        // ======== 配置并打开串口 ========
        private void ConfigureAndOpenSerialPort()
        {
            serialPort.PortName = com.SelectedItem.ToString(); // 设置串口号
            serialPort.BaudRate = int.Parse(Baud_rate.SelectedItem.ToString()); // 设置波特率
            serialPort.Open();
        }

        // ======== 更新开始按钮状态 ========
        private void UpdateStartButtonState(bool isOpen)
        {
            Btn_start.Content = isOpen ? "关闭" : "开始"; // 更新按钮文本
            Baud_rate.IsEnabled = !isOpen; // 禁用波特率选择
            com.IsEnabled = !isOpen; // 禁用串口选择
            bool areControlsDisabled = isOpen; // 是否禁用其他控件
            Save_1Hz.IsEnabled = !areControlsDisabled; // 更新保存1Hz按钮状态
            Save_100Hz.IsEnabled = !areControlsDisabled; // 更新保存100Hz按钮状态
            Draw_100Hz.IsEnabled = !areControlsDisabled; // 更新100Hz绘图按钮状态
            Draw_1Hz.IsEnabled = !areControlsDisabled; // 更新1Hz绘图按钮状态
            sample_time.IsEnabled = false; // 禁用采样时间文本框
            string message =
                "********************************************************************\n" +
                "正在检测采集配置...\n" +
                $"设置端口为: {serialPort.PortName}\n" +
                $"设置波特率为: {serialPort.BaudRate}\n" +
                $"采样时间为: {sample_time.Text}s\n" +
                "此端口已经打开！ \n正在采集数据... \n";
            UpdateState(message); // 合并所有状态信息为一个字符串，一次性更新所有状态信息
        }

        // ======== 在串口关闭时启用控件 ========
        private void EnableControlsOnSerialPortClose()
        {
            Baud_rate.IsEnabled = true; // 启用波特率选择
            com.IsEnabled = true; // 启用串口选择
            Save_1Hz.IsEnabled = true; // 启用保存1Hz按钮
            Save_100Hz.IsEnabled = true; // 启用保存100Hz按钮
            Draw_100Hz.IsEnabled = true; // 启用100Hz绘图按钮
            Draw_1Hz.IsEnabled = true; // 启用1Hz绘图按钮
            sample_time.IsEnabled = true; // 重新启用采样时间文本框
        }

        // ======== 100Hz绘图选择事件 ========
        private void Draw_100Hz_Checked(object sender, RoutedEventArgs e)
        {
            isDraw100HzSelected = true; // 启用100Hz绘图
            isDraw1HzSelected = false; // 禁用1Hz绘图
            UpdateState($"设置采样频率为:100Hz\n");
        }

        // ======== 1Hz绘图选择事件 ========
        private void Draw_1Hz_Checked(object sender, RoutedEventArgs e)
        {
            isDraw1HzSelected = true; // 启用1Hz绘图
            isDraw100HzSelected = false; // 禁用100Hz绘图
            UpdateState($"设置采样频率为:1Hz\n");
        }

        // ======== 1Hz保存选择事件 ========
        private void Save_1Hz_Checked(object sender, RoutedEventArgs e)
        {
            Btn_1Hz.IsEnabled = true; // 启用选择路径按钮
            save_path_1Hz.IsEnabled = true; // 启用路径文本框
            isSave1HzEnabled = true; // 启用1Hz保存
        }

        // ======== 1Hz保存取消事件 ========
        private void Save_1Hz_Unchecked(object sender, RoutedEventArgs e)
        {
            DisableSaveControls(Save_1Hz, Btn_1Hz, save_path_1Hz); // 禁用保存控件
            filePath1Hz = string.Empty; // 清空保存路径
            isSave1HzEnabled = false; // 禁用1Hz保存
        }

        // ======== 100Hz保存选择事件 ========
        private void Save_100Hz_Checked(object sender, RoutedEventArgs e)
        {
            Btn_100Hz.IsEnabled = true; // 启用选择路径按钮
            save_path_100Hz.IsEnabled = true; // 启用路径文本框
            isSave100HzEnabled = true; // 启用100Hz保存
        }

        // ======== 100Hz保存取消事件 ========
        private void Save_100Hz_Unchecked(object sender, RoutedEventArgs e)
        {
            DisableSaveControls(Save_100Hz, Btn_100Hz, save_path_100Hz); // 禁用保存控件
            filePath100Hz = string.Empty; // 清空保存路径
            isSave100HzEnabled = false; // 禁用100Hz保存
        }

        // ======== 禁用保存控件 ========
        private void DisableSaveControls(CheckBox saveCheckBox, Button button, TextBox textBox)
        {
            button.IsEnabled = false; // 禁用按钮
            textBox.IsEnabled = false; // 禁用文本框
            saveCheckBox.IsChecked = false; // 取消选中复选框
        }

        // ======== 选择1Hz数据保存路径 ========
        private void Btn_1Hz_Click(object sender, RoutedEventArgs e)
        {
            SelectSaveFilePath(ref filePath1Hz, save_path_1Hz, "1Hz_数据", "1Hz"); // 选择保存路径
        }

        // ======== 选择100Hz数据保存路径 ========
        private void Btn_100Hz_Click(object sender, RoutedEventArgs e)
        {
            SelectSaveFilePath(ref filePath100Hz, save_path_100Hz, "100Hz_数据", "100Hz"); // 选择保存路径
        }

        // ======== 关联所有X轴 ========
        private void LinkAllXAxes()
        {
            // 获取所有图表的引用
            var plots = plotParameterToControlMap.Values.ToList();

            if (plots.Count == 0)
                return;

            // 将所有图表的X轴相互关联
            for (int i = 0; i < plots.Count; i++)
            {
                for (int j = 0; j < plots.Count; j++)
                {
                    if (i != j)
                    {
                        plots[i].Plot.Axes.Link(plots[j], x: true, y: false);
                    }
                }
            }
        }

        // ======== 自动按钮选中事件 ========
        private void Cb_AutoFit_Checked(object sender, RoutedEventArgs e)
        {
            isAutoFitEnabled = true; // 启用自动缩放
        }

        // ======== 自动按钮取消选中事件 ========
        private void Cb_AutoFit_Unchecked(object sender, RoutedEventArgs e)
        {
            isAutoFitEnabled = false; // 禁用自动缩放
        }

        // ======== 误差补偿窗口 ========
        private void EC_btn_Click(object sender, RoutedEventArgs e)
        {
            if (_ecWindow == null || !_ecWindow.IsVisible)
            {
                _ecWindow = new ECWindow(serialPort);
                _ecWindow.Show();
            }
            else
            {
                _ecWindow.Activate();
            }
        }

        //********窗口关闭与清理模块********
        // ======== 关闭串口 ========
        private void CloseSerialPort()
        {
            if (serialPort != null && serialPort.IsOpen)
            {
                serialPort.Close(); // 关闭串口
                UpdateState("端口关闭,采集停止.\n");
            }
        }

        // ======== 关闭文件写入流 ========
        private void CloseFileWriters()
        {
            writer1Hz?.Close(); // 关闭1Hz写入流
            writer100Hz?.Close(); // 关闭100Hz写入流
        }

        // ======== 窗口关闭事件 ========
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            // 关闭ECWindow
            if (_ecWindow != null && _ecWindow.IsVisible)
            {
                _ecWindow.Close();
            }

            CloseSerialPort(); // 关闭串口
            cancellationTokenSource?.Cancel(); // 取消任务
            CloseFileWriters(); // 关闭文件写入流
            isProcessing = false; // 停止数据处理
            isReceiving = false; // 停止数据接收

            // 等待线程结束
            if (processingThread != null && processingThread.IsAlive)
                processingThread.Join();

            if (receiveThread != null && receiveThread.IsAlive)
                receiveThread.Join();
        }
    }
}