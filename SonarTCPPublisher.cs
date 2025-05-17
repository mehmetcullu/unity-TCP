using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

public class SonarTcpPublisher : MonoBehaviour
{
    public SonarRayCast sonarRayCast; // Inspector'dan atayın

    private TcpListener tcpListener;
    private Thread listenerThread;
    private bool running = false;

    void Start()
    {
        tcpListener = new TcpListener(IPAddress.Any, 5556);
        tcpListener.Start();

        running = true;
        listenerThread = new Thread(ClientHandler);
        listenerThread.IsBackground = true;
        listenerThread.Start();
    }

    void ClientHandler()
    {
        while (running)
        {
            if (tcpListener.Pending())
            {
                TcpClient client = tcpListener.AcceptTcpClient();
                NetworkStream stream = client.GetStream();

                while (client.Connected && running)
                {
                    if (sonarRayCast != null)
                    {
                        float[] hits = sonarRayCast.Hits;
                        if (hits != null && hits.Length > 0)
                        {
                            string message = string.Join(",", hits);
                            byte[] data = Encoding.UTF8.GetBytes(message + "\n");
                            stream.Write(data, 0, data.Length);
                            stream.Flush();
                        }
                    }
                    Thread.Sleep(100); // 10Hz
                }

                client.Close();
            }
            else
            {
                Thread.Sleep(100);
            }
        }
    }

    void OnDestroy()
    {
        running = false;
        tcpListener.Stop();

        if (listenerThread != null && listenerThread.IsAlive)
            listenerThread.Join();
    }
}