using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json.Linq;

public class TestReceiver : MonoBehaviour
{
    // Start is called before the first frame update
    [SerializeField] private TextAsset driveMessageJson;
    [SerializeField] private UdpController udpController;  // assign in Inspector
    private JObject driveMessage;
    void Start()
    {
        StartCoroutine(RunAtInterval());
        udpController.ConfigureSubscription("drive_topic","rover2_control_interface/msg/DriveCommandMessage");
    }
    
    IEnumerator RunAtInterval()
    {
        while (true) // Or some condition
        {
            Debug.Log(udpController.GetLatestMessage("drive_topic"));
            
            yield return new WaitForSeconds(1f); // Wait 1 second
        }
    }
}
