using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Net.Sockets;
using System.Net;
using System.Text;
using Newtonsoft.Json.Linq;
using System;
using UnityEditor.VersionControl;
using Cysharp.Threading.Tasks;

public class UdpController : MonoBehaviour
{
    public static UdpController inst;
    private UdpClient udpClient;           // for sending
    private UdpClient udpReceiveClient;    // for receiving
    private UdpClient configClient;

    private UdpClient srvClient;
    private UdpClient srvServer;

    
    private IPEndPoint serverEndPoint;
    private IPEndPoint receiveEndPoint;

    private IPEndPoint configEndPoint;

    private IPEndPoint srvEndPoint;

    private IPEndPoint srvServerPoint;


    private bool isConnected;
    public bool disconnected;

    [Header("UDP Configuration")]
    public string serverIP = "127.0.0.1";
    private int UDPSendPort = 65434;
    private int UDPReceivePort = 65435;

    private int UDPConfigPort = 65436;

    private int UDPConfigClientReq = 65437;

    private int UDPServerPort = 65438;

    

    [Header("Connection Status")]
    public bool showDebugLogs = true;

    // Store latest messages per topic
    private Dictionary<string, JObject> latestMessages = new Dictionary<string, JObject>();

    public void Reconnect()
    {
        Start();
    }

    void Start()
    {
        inst = this;

        try
        {
            // Main send client
            udpClient = new UdpClient();
            serverEndPoint = new IPEndPoint(IPAddress.Parse(serverIP), UDPSendPort);

            // Receive client
            udpReceiveClient = new UdpClient(UDPReceivePort);
            receiveEndPoint = new IPEndPoint(IPAddress.Any, 0);

            // Config client
            configClient = new UdpClient();
            configEndPoint = new IPEndPoint(IPAddress.Parse(serverIP), UDPConfigPort);

            srvClient = new UdpClient();
            srvEndPoint = new IPEndPoint(IPAddress.Parse(serverIP), UDPConfigClientReq);

            srvServer = new UdpClient(UDPServerPort);
            srvServerPoint = new IPEndPoint(IPAddress.Any, 0);

            isConnected = true;
            disconnected = false;

            if (showDebugLogs)
                Debug.Log($"UDP Controller initialized: send:{serverIP}:{UDPSendPort}, receive:{UDPReceivePort}, client:{UDPConfigPort}");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Could not initialize UDP Controller: {e.Message}");
            disconnected = true;
            isConnected = false;
        }
    }




    void Update()
    {
        // Continuously check for messages
        if (udpReceiveClient != null && udpReceiveClient.Available > 0)
        {
            try
            {
                byte[] data = udpReceiveClient.Receive(ref receiveEndPoint);
                string jsonString = Encoding.UTF8.GetString(data);
                JObject message = JObject.Parse(jsonString);

                // Update latest messages dictionary
                string topic = message["topic"]?.ToString();
                if (!string.IsNullOrEmpty(topic))
                {

                    latestMessages[topic] = message;
                    if (showDebugLogs)
                        Debug.Log($"Updated latest message for topic: {topic}");
                }
            }
            catch (Exception e)
            {
                Debug.LogWarning($"Failed to receive or parse UDP message: {e.Message}");
            }
        }
    }


    async UniTask<JObject> checkService(int maxAttempts = 50, float delayBetweenAttempts = 100)
    {
        for (int i = 0; i < maxAttempts; i++)
        {
            try
            {
                if (srvServer != null)
                {
                    UdpReceiveResult result = await srvServer.ReceiveAsync().AsUniTask();
                    string jsonString = Encoding.UTF8.GetString(result.Buffer);
                    JObject message = JObject.Parse(jsonString);

                    Debug.Log($"Received from {result.RemoteEndPoint.Address}:{result.RemoteEndPoint.Port}: {jsonString}");
                    return message;
                }
            }
            catch (Exception e)
            {
                Debug.LogWarning($"Failed to receive or parse UDP message: {e.Message}");
            }

            await UniTask.Delay(TimeSpan.FromSeconds(delayBetweenAttempts));
        }

        Debug.LogWarning("checkServiceAsync timed out: no message received");
        return new JObject();
    }


    public void ConfigureSubscription(string topic, string msgType)
    {
        string message = topic + ";" + msgType;
        if (showDebugLogs)
            Debug.Log($"Sending config UDP Message: {message}");

        if (!isConnected || configClient == null)
        {
            Debug.LogWarning("UDP config connection is not established. Attempting reconnect.");
            Start();
            if (!isConnected || configClient == null)
            {
                Debug.LogWarning("UDP config reconnect failed. Canceling publish.");
                return;
            }
        }

        try
        {
            byte[] dataToSend = Encoding.UTF8.GetBytes(message);
            configClient.Send(dataToSend, dataToSend.Length, configEndPoint);

            disconnected = false;
        }
        catch (System.Exception e)
        {
            Debug.LogWarning($"Error while sending UDP config data: {e.Message}");
            disconnected = true;
            isConnected = false;
        }
    }



    public void PublishMessage(string message)
    {
        if (showDebugLogs)
            Debug.Log($"Sending UDP Message: {message}");

        if (!isConnected || udpClient == null)
        {
            Debug.LogWarning("UDP connection is not established. Attempting reconnect.");
            Start();
            if (!isConnected || udpClient == null)
            {
                Debug.LogWarning("UDP reconnect failed. Canceling publish.");
                return;
            }
        }

        try
        {
            byte[] dataToSend = Encoding.UTF8.GetBytes(message);
            udpClient.Send(dataToSend, dataToSend.Length, serverEndPoint);

            disconnected = false;
        }
        catch (System.Exception e)
        {
            Debug.LogWarning($"Error while sending UDP control data: {e.Message}");
            disconnected = true;
            isConnected = false;
        }
    }

    public async UniTask<JObject> PublishClientReq(string message)
    {
        if (showDebugLogs)
        Debug.Log($"Sending UDP Client Req: {message}");
        JObject SrvReturn = new JObject();

        if (!isConnected || udpClient == null)
        {
            Debug.LogWarning("UDP connection is not established. Attempting reconnect.");
            Start();
            if (!isConnected || udpClient == null)
            {
                Debug.LogWarning("UDP reconnect failed. Canceling publish.");
                return SrvReturn;
            }
        }

        try
        {
            byte[] dataToSend = Encoding.UTF8.GetBytes(message);
            udpClient.Send(dataToSend, dataToSend.Length, srvEndPoint);

            disconnected = false;
            SrvReturn = await checkService();
            return SrvReturn;
        }
        catch (System.Exception e)
        {
            Debug.LogWarning($"Error while sending UDP client req: {e.Message}");
            disconnected = true;
            isConnected = false;
            return SrvReturn;
        }
    }
    

    // Get the latest message for a topic
    public JObject GetLatestMessage(string topic)
    {
        latestMessages.TryGetValue(topic, out JObject msg);
        return msg;
    }

    void OnDestroy()
    {
        isConnected = false;

        if (udpClient != null)
        {
            try { udpClient.Close(); } catch (Exception e) { Debug.LogWarning($"Error closing UDP client: {e.Message}"); }
            udpClient = null;
        }

        if (udpReceiveClient != null)
        {
            try { udpReceiveClient.Close(); } catch (Exception e) { Debug.LogWarning($"Error closing UDP receive client: {e.Message}"); }
            udpReceiveClient = null;
        }

        if (showDebugLogs)
            Debug.Log("UDP Control Controller closed.");
    }

    void OnApplicationPause(bool pauseStatus)
    {
        if (pauseStatus)
        {
            isConnected = false;
        }
        else if (!isConnected)
        {
            Start();
        }
    }
}
