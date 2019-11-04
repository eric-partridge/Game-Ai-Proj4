using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// MapStateManager is the place to keep a succession of events or "states" when building 
/// a multi-step AI demo. Note that this is a way to manage 
/// 
/// State changes could happen for one of two reasons:
///     when the user has pressed a number key 0..9, desiring a new phase
///     when something happening in the game forces a transition to the next phase
/// 
/// One use will be for AI demos that are switched up based on keyboard input. For that, 
/// the number keys 0..9 will be used to dial in whichever phase the user wants to see.
/// </summary>

public class FieldMapManager : MonoBehaviour {
    // Set prefabs
    public GameObject PlayerPrefab;     // You, the player
    public GameObject HunterPrefab;     // Agent doing chasing
    public GameObject WolfPrefab;       // Agent getting chased
    public GameObject RedPrefab;        // Red Riding Hood, or just "team red"
    public GameObject BluePrefab;       // "team blue"
    public GameObject TreePrefab;       // New for Assignment #2

    public NPCController house;         // for future use

    public Camera mainCam;

    // Set up to use spawn points. Can add more here, and also add them to the 
    // Unity project. This won't be a good idea later on when you want to spawn
    // a lot of agents dynamically, as with Flocking and Formation movement.

    public GameObject spawner1;
    public Text SpawnText1;
    public GameObject spawner2;
    public Text SpawnText2;
    public GameObject spawner3;
    public Text SpawnText3;
    public Text KeyText;

    public int TreeCount;

    public List<GameObject> spawnedNPCs;   // When you need to iterate over a number of agents.
    public List<GameObject> spawnedWolfs;
    public List<GameObject> spawnedReds;
    private List<GameObject> trees;

    private int currentPhase = -1;           // This stores where in the "phases" the game is.
    private int previousPhase = -1;          // The "phases" we were just in

    //public int Phase => currentPhase;

    public GameObject[] PathRed;
    public GameObject[] PathWolf;

    LineRenderer line;
    public Text narrator;                   // 

    public GameObject leaderRef;          //this is used to store the leader reference

    // Use this for initialization. Create any initial NPCs here and store them in the 
    // spawnedNPCs list. You can always add/remove NPCs later on.

    void Start()
    {
        narrator.text = "This is the place to mention major things going on during the demo, the \"narration.\"";

        spawnedNPCs = new List<GameObject>();
        //spawnedNPCs.Add(RedPrefab);
        //spawnedNPCs.Add(WolfPrefab);

        /* Code for part 2
        */
        //modify this text and the same one in mapPhase 0 to change the discription
        narrator.text = "Press 1 for flocking\nPress 2 to Reset\n";
    }

    /// <summary>
    /// This is where you put the code that places the level in a particular phase.
    /// Unhide or spawn NPCs (agents) as needed, and give them things (like movements)
    /// to do. For each case you may well have more than one thing to do.
    /// </summary>
    private void Update()
    {
        string inputstring = Input.inputString;
        int num = -1;

        if (inputstring.Length > 0)
        {
            Debug.Log(inputstring);


            if (Int32.TryParse(inputstring, out num))
            {
                if (num != currentPhase)
                {
                    previousPhase = currentPhase;
                    currentPhase = num;
                }
            }

            else if (inputstring[0] == 'K' || inputstring[0] == 'k')
            {
                if (KeyText.text.Length == 0)
                {
                    KeyText.text = "m: follow mouse\n" +
                        "c: Cone check\n" +
                        "p: Part2 avoidance\n"+
                        "a: Start PathFollowing" +
                        "1: show Part 1 flocking algo" +
                        "2: Clear all";      
                        
                }
                else
                {
                    KeyText.text = "";
                }
            }
            /*else if (inputstring[0] == 'P' || inputstring[0] == 'p')
            {
                currentPhase = 10;
            }*/
            else if (inputstring[0] == 'W' || inputstring[0] == 'w')
            {
                currentPhase = 11;
            }
            else if (inputstring[0] == 'A' || inputstring[0] == 'a')
            {
                for (int i = 0; i < 6; i++)
                {
                    spawnedNPCs.Add(SpawnItem(spawner1, RedPrefab, null, null, -1));
                    spawnedNPCs[i].GetComponent<SteeringBehavior>().Path = PathRed;
                    spawnedReds.Add(spawnedNPCs[i]);
                }

                for (int i = 6; i < 12; i++)
                {
                    spawnedNPCs.Add(SpawnItem(spawner3, WolfPrefab, null, null, -1));
                    spawnedNPCs[i].GetComponent<SteeringBehavior>().Path = PathWolf;
                    spawnedWolfs.Add(spawnedNPCs[i]);
                }
                for (int i = 0; i < 12; i++)
                {
                    spawnedNPCs[i].GetComponent<SteeringBehavior>().startPathFollowing = true;
                    spawnedNPCs[i].GetComponent<SteeringBehavior>().partTwo = true;
                }

            }
            else if (inputstring[0] == 'P' || inputstring[0] == 'p')
            {
                currentPhase = 13;
                for (int i = 0; i < 12; i++)
                {
                    spawnedNPCs[i].GetComponent<SteeringBehavior>().collisionPrediction = true;
                }

            }
            else if (inputstring[0] == 'C' || inputstring[0] == 'c')
            {
                currentPhase = 14;
                for(int i = 0; i < 12; i++)
                {
                    spawnedNPCs[i].GetComponent<SteeringBehavior>().coneCheck = true;
                }
            }
            else if(inputstring[0] == 'm' || inputstring[0] == 'M')
            {
                for(int i = 0; i < spawnedNPCs.Count; i++)
                {
                    NPCController nc = spawnedNPCs[i].GetComponent<NPCController>();
                    nc.seekMousePos = !nc.seekMousePos;
                }
            }
        }

        switch (currentPhase)
        {
            case 0:
                //idle
                narrator.text = "Press 1 for flocking\nPress 2 to Reset.\n";
                break;
            case 1:
                
                GameObject _red = SpawnItem(spawner3, RedPrefab, null, SpawnText1, 0);
                spawnedNPCs.Add(_red);
                _red.GetComponent<NPCController>().seekMousePos = true;
                _red.GetComponent<NPCController>().maxSpeed = 8f;
                for(int i = 0; i < 20; i++)
                {
                    GameObject npc = SpawnItem(spawner1, HunterPrefab, _red.GetComponent<NPCController>(), SpawnText3, 1);
                    spawnedNPCs.Add(npc);
                }

                previousPhase = currentPhase;
                currentPhase = 0;
                break;
            case 2:
                while(spawnedNPCs.Count != 0)
                {
                    GameObject tempRef = spawnedNPCs[0];
                    spawnedNPCs.RemoveAt(0);
                    Destroy(tempRef);
                }

                previousPhase = currentPhase;
                currentPhase = 0;
                break;
          
        }
    }


    


    // ... Etc. Etc.

    /// <summary>
    /// SpawnItem placess an NPC of the desired type into the game and sets up the neighboring 
    /// floating text items nearby (diegetic UI elements), which will follow the movement of the NPC.
    /// </summary>
    /// <param name="spawner"></param>
    /// <param name="spawnPrefab"></param>
    /// <param name="target"></param>
    /// <param name="spawnText"></param>
    /// <param name="phase"></param>
    /// <returns></returns>
    private GameObject SpawnItem(GameObject spawner, GameObject spawnPrefab, NPCController target, Text spawnText, int phase)
    {
        Vector3 size = spawner.transform.localScale;
        Vector3 position = spawner.transform.position + new Vector3(UnityEngine.Random.Range(-size.x / 2, size.x / 2), 0, UnityEngine.Random.Range(-size.z / 2, size.z / 2));
        GameObject temp = Instantiate(spawnPrefab, position, Quaternion.identity);
        if (target)
        {
            temp.GetComponent<SteeringBehavior>().target = target;
        }
        temp.GetComponent<NPCController>().label = spawnText;
        temp.GetComponent<NPCController>().phase = phase;
        Camera.main.GetComponent<CameraController>().player = temp;
        return temp;
    }

    /// <summary>
    /// SpawnTrees will randomly place tree prefabs all over the map. The diameters
    /// of the trees are also varied randomly.
    /// 
    /// Note that it isn't particularly smart about this (yet): notably, it doesn't
    /// check first to see if there is something already there. This should get fixed.
    /// </summary>
    /// <param name="numTrees">desired number of trees</param>
    private void SpawnTrees(int numTrees)
    {
        float MAX_X = 25;  // Size of the map; ideally, these shouldn't be hard coded
        float MAX_Z = 20;
        float less_X = MAX_X - 1;
        float less_Z = MAX_Z - 1;

        float diameter;

        for (int i = 0; i < numTrees; i++)
        {
            //Vector3 size = spawner.transform.localScale;
            Vector3 position = new Vector3(UnityEngine.Random.Range(-less_X, less_X), 0, UnityEngine.Random.Range(-less_Z, less_Z));
            GameObject temp = Instantiate(TreePrefab, position, Quaternion.identity);

            // diameter will be somewhere between .2 and .7 for both X and Z:
            diameter = UnityEngine.Random.Range(0.2F, 0.7F);
            temp.transform.localScale = new Vector3(diameter, 1.0F, diameter);

            trees.Add(temp);

        }
    }

    private void DestroyTrees()
    {
        GameObject temp;
        for (int i = 0; i < trees.Count; i++)
        {
            temp = trees[i];
            Destroy(temp);
        }
        // Following this, write whatever methods you need that you can bolt together to 
        // create more complex movement behaviors.
    }
    private void SpawnWolf()
    {
        narrator.text = "The Wolf appears. Most wolves are ferocious, but this one is docile.";
        spawnedNPCs.Add(SpawnItem(spawner2, WolfPrefab, null, SpawnText2, 4));
    }

    private void Meeting1()
    {
        if (currentPhase == 0)
        {
            spawnedNPCs[0].GetComponent<SteeringBehavior>().target = spawnedNPCs[1].GetComponent<NPCController>();
            spawnedNPCs[1].GetComponent<SteeringBehavior>().target = spawnedNPCs[0].GetComponent<NPCController>();
            SetArrive(spawnedNPCs[0]);
            SetArrive(spawnedNPCs[1]);
        }
    }

    private void SpawnWolf2()
    {
        narrator.text = "The Wolf looks for shelter, and spots little Red.";
        spawnedNPCs.Add(SpawnItem(spawner3, WolfPrefab, spawnedNPCs[2].GetComponent<NPCController>(), SpawnText1, 1));
        spawnedNPCs[3].GetComponent<NPCController>().label.enabled = true;
    }

    private void Meeting2()
    {
        narrator.text = "The two converse, and little Red directs the Wolf to her house.";
        spawnedNPCs[2].GetComponent<NPCController>().DestroyPoints();
        spawnedNPCs[2].GetComponent<NPCController>().phase = 5;
        spawnedNPCs[3].GetComponent<SteeringBehavior>().target = house;
        spawnedNPCs[3].GetComponent<NPCController>().phase = 1; ;
        Invoke("SpawnHunter", 10);
    }

    private void SpawnHunter()
    {
        narrator.text = "The Hunter arrives, determined to catch the killer. He spots a house and moves accordingly.";
        spawnedNPCs.Add(SpawnItem(spawner3, HunterPrefab, house, SpawnText2, 1));
        spawnedNPCs[4].GetComponent<NPCController>().label.enabled = true;
    }

    private void End()
    {
        narrator.text = "Days later, reports come in. The killer is still at large, but police have found one clue on its identity. "
            + "A little red hood. END";
        currentPhase++;
    }

    private void SetArrive(GameObject character)
    {

        character.GetComponent<NPCController>().phase = 3;
        character.GetComponent<NPCController>().DrawConcentricCircle(character.GetComponent<SteeringBehavior>().slowRadiusL);
    }

    /*private void CreatePath()
    {
        line = GetComponent<LineRenderer>();
        line.positionCount = Path.Length;
        for (int i = 0; i < Path.Length; i++)
        {
            line.SetPosition(i, Path[i].transform.position);
        }
    }*/

    void OnDrawGizmosSelected()
    {
        Gizmos.color = new Color(1, 0, 0, 0.5f);
        Gizmos.DrawCube(spawner1.transform.position, spawner1.transform.localScale);
        Gizmos.DrawCube(spawner2.transform.position, spawner2.transform.localScale);
        Gizmos.DrawCube(spawner3.transform.position, spawner3.transform.localScale);
    }

    public List<GameObject> AllNPCs()
    {
        return spawnedNPCs;
    }
}
