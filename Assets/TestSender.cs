using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json.Linq;

public class TestSender : MonoBehaviour
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
            driveMessage = JObject.Parse(driveMessageJson.text);
            driveMessage["topic"] = "drive_topic";
            driveMessage["msgType"] = "DriveCommandMessage";
            driveMessage["data"]["controller_present"] = false;
            driveMessage["data"]["drive_twist"]["linear"]["x"] = 2.5;
            driveMessage["data"]["drive_twist"]["linear"]["y"] = -0.8;
            

            string msg = driveMessage.ToString();
            Debug.Log("Drive message:\n" + msg);

            udpController.PublishMessage(msg);
            yield return new WaitForSeconds(1f); // Wait 1 second
        }
    }
}
