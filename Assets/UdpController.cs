using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Net.Sockets;
using System.Net;
using System.Text;
using Newtonsoft.Json.Linq;
using System;

public class UdpController : MonoBehaviour
{
    public static UdpController inst;
    private UdpClient udpClient;           // for sending
    private UdpClient udpReceiveClient;    // for receiving
    private IPEndPoint serverEndPoint;
    private IPEndPoint receiveEndPoint;
    private bool isConnected;
    public bool disconnected;

    [Header("UDP Configuration")]
    public string serverIP = "127.0.0.1";
    public int UDPSendPort = 65434;
    public int UDPReceivePort = 65435;

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
            // Send client
            udpClient = new UdpClient();
            serverEndPoint = new IPEndPoint(IPAddress.Parse(serverIP), UDPSendPort);

            // Receive client bound to the receive port
            udpReceiveClient = new UdpClient(UDPReceivePort);
            receiveEndPoint = new IPEndPoint(IPAddress.Any, 0);

            isConnected = true;
            disconnected = false;

            if (showDebugLogs)
                Debug.Log($"UDP Control Controller initialized for send:{serverIP}:{UDPSendPort}, receive:{UDPReceivePort}");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Could not initialize UDP Control Controller: {e.Message}");
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
