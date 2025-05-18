using System;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

public class SonarTcpSubscriber : MonoBehaviour
{
    public string serverIp = "127.0.0.1"; // Python server IP
    public int port = 5556;

    private TcpClient client;
    private NetworkStream stream;
    private Thread clientThread;
    private bool running = false;

    // Son alınan veri
    public float[] latestHits;

    void Start()
    {
        clientThread = new Thread(RunClient);
        clientThread.IsBackground = true;
        running = true;
        clientThread.Start();
    }

    void RunClient()
    {
        try
        {
            client = new TcpClient(serverIp, port);
            stream = client.GetStream();
            byte[] buffer = new byte[1024];
            StringBuilder dataBuffer = new StringBuilder();

            while (running && client.Connected)
            {
                int bytesRead = stream.Read(buffer, 0, buffer.Length);
                if (bytesRead == 0) break;

                string data = Encoding.UTF8.GetString(buffer, 0, bytesRead);
                dataBuffer.Append(data);

                while (dataBuffer.ToString().Contains("\n"))
                {
                    string fullData = dataBuffer.ToString();
                    int index = fullData.IndexOf("\n");
                    string line = fullData.Substring(0, index).Trim();
                    dataBuffer.Remove(0, index + 1);

                    // Parse line to float array
                    string[] parts = line.Split(',');
                    float[] hits = new float[parts.Length];
                    for (int i = 0; i < parts.Length; i++)
                    {
                        if (float.TryParse(parts[i], out float val))
                            hits[i] = val;
                        else
                            hits[i] = -1f; // Parsing hatasında -1 koy
                    }

                    latestHits = hits;
                    Debug.Log("Gelen veri: " + string.Join(", ", hits));
                }
            }
        }
        catch (Exception ex)
        {
            Debug.LogError("TCP Client hatası: " + ex.Message);
        }
    }

    void OnDestroy()
    {
        running = false;
        stream?.Close();
        client?.Close();
        clientThread?.Join();
    }
}