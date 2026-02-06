using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json.Linq;

public class TestSender : MonoBehaviour
{
    // Start is called before the first frame update
    [SerializeField] private TextAsset driveMessageJson;

    [SerializeField] private TextAsset testServiceJson;

    [SerializeField] private UdpController udpController;  // assign in Inspector
    private JObject driveMessage;

    private JObject testService;

    private JObject response;
    async void Start()
    {
        StartCoroutine(RunAtInterval());
        testService = JObject.Parse(testServiceJson.text);

        testService["service"]="test_srv";
        testService["request"]["data"]=true;
        string msg = testService.ToString();
        response = await udpController.PublishClientReq(msg);


    }
    
    IEnumerator RunAtInterval()
    {
        
        while (true) // Or some condition
        {
            //Get parsed json, assigned in insepector
            driveMessage = JObject.Parse(driveMessageJson.text);
            //Assign relevant datafields for message
            driveMessage["topic"] = "drive_topic";
            driveMessage["data"]["controller_present"] = false;
            driveMessage["data"]["drive_twist"]["linear"]["x"] = 2.5;
            driveMessage["data"]["drive_twist"]["linear"]["y"] = -0.8;
            
            //Convert to string for sending across UDP pipe
            string msg = driveMessage.ToString();
            //Send message to ROS2 negotiator
            udpController.PublishMessage(msg);
            

            yield return new WaitForSeconds(1f); // Wait 1 second
        }
        
    }
}
