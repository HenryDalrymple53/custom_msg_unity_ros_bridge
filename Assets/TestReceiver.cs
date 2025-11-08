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
