using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// This is the place to put all of the various steering behavior methods we're going
/// to be using. Probably best to put them all here, not in NPCController.
/// </summary>

public class SteeringBehavior : MonoBehaviour {

    // The agent at hand here, and whatever target it is dealing with
    public NPCController agent;
    public NPCController target;

    // Below are a bunch of variable declarations that will be used for the next few
    // assignments. Only a few of them are needed for the first assignment.

    // For pursue and evade functions
    public float maxPrediction;
    public float maxAcceleration;

    // For arrive function
    public float maxSpeed;
    public float targetRadiusL;
    public float slowRadiusL;
    public float timeToTarget;
    public float targetSpeedL;

    // For Face function
    public float maxRotation;
    public float maxAngularAcceleration;
    public float targetRadiusA;
    public float slowRadiusA;

    // For wander function
    public float wanderOffset;
    public float wanderRadius;
    public float wanderRate;
    private float wanderOrientation;
    public Transform wanderTarget;

    // Holds the path to follow
    public GameObject[] Path;
    public int current = 0;

    // For wall avoidance
    public float raycastDistance;
    public float avoidanceMultipler;

    public bool startPathFollowing = false;
    public bool collisionPrediction = false;
    public bool coneCheck = false;
    public bool lrrhWander = false;

    private Transform startPos;
    private Transform endPos;
    float startTime;
    float journeyLength;

    public List<GameObject> enemies;

    public float coneThreshold;


    //for collision prediction and detection
    public struct wanderSteering
    {
        public float angular;
        public Vector3 linear;
    }

    public struct wallAvoidanceSteering
    {
        public float angular;
        public Vector3 linear;
        public bool needAvoidance;
    }

    protected void Start() {
        agent = GetComponent<NPCController>();
        wanderOrientation = agent.orientation;
    }

    public void FixedUpdate()
    {
        //ensure it is time to start path following and you havent reached the end
        if (current < Path.Length && startPathFollowing)
        {
            //get start and end points for section of path
            startPos = agent.transform;
            endPos = Path[current].transform;



            //create struct to hold output
            wanderSteering ret;


            if (coneCheck)
            {
                //perform cone check
                wanderSteering coneCheckRet = ConeCheck();
                if (coneCheckRet.linear != Vector3.zero && Mathf.Abs(coneCheckRet.angular) > Mathf.Epsilon)
                {
                    ret.linear = coneCheckRet.linear;
                    ret.angular = coneCheckRet.angular;
                }
                else
                {
                    //calculate angular direction
                    ret.angular = Align(endPos.position);
                }
            }
            else
            {
                //calculate angular direction
                ret.angular = Align(endPos.position);
            }

            //check for collision detection otherwise keep linear normal
            if (collisionPrediction)
            {
                Vector3 foundCollision = CollisionDetectAndAvoid();
                if (foundCollision == Vector3.zero)
                {
                    ret.linear = maxAcceleration * new Vector3(Mathf.Sin(agent.orientation), 0, Mathf.Cos(agent.orientation));
                }
                else
                {
                    ret.linear = foundCollision;
                }
            }
            else
            {
                ret.linear = maxAcceleration * new Vector3(Mathf.Sin(agent.orientation), 0, Mathf.Cos(agent.orientation));
            }


            agent.GetComponent<NPCController>().update(ret.linear, ret.angular, Time.deltaTime);

            //if you reached endPos advance to next point in path
            if ((agent.position - endPos.position).magnitude <= targetRadiusL)
            {
                current += 1;
            }
        }
        /*//othewise dont move player
        else if (current >= Path.Length)
        {
            agent.GetComponent<NPCController>().UpdateMovement(Vector3.zero, 0f, Time.deltaTime);
        }*/
    }

    public wanderSteering DynamicEvade()
    {
        wanderSteering result;
        float distance = GetDistance();
        float agentSpeed = agent.velocity.magnitude;
        float prediction = maxPrediction;

        //check could agent get target in maxPrediction time
        if (agentSpeed > distance / maxPrediction)
        {
            prediction = distance / agentSpeed;
        }

        //get the future location
        Vector3 futureLocation = target.position + (target.velocity * prediction);

        //seek for future location
        Vector3 futureAccleration = agent.position - futureLocation;

        //clip to max acceleration
        if (futureAccleration.magnitude > maxAcceleration)
        {
            futureAccleration = futureAccleration.normalized * maxAcceleration;
        }
        //agent.DrawCircle(futureAccleration + agent.position, 1.0f);

        result.linear = futureAccleration;
        result.angular = Align(futureAccleration + agent.position);
        return result;
    }

    //this function returns the distance between agent and target
    private float GetDistance()
    {
        if (lrrhWander)
        {
            return (wanderTarget.position - agent.position).magnitude;
        }
        else if (target == null)
        {
            return (endPos.position - agent.position).magnitude;
        }
        else
        {
            return (target.position - agent.position).magnitude;
        }

        //return result;
    }

    //this function returns the vector from agent to target
    private Vector3 GetDirectionVec()
    {
        if (lrrhWander)
        {
            return wanderTarget.position - agent.position;
        }
        else if (target == null)
        {
            return endPos.position - agent.position;
        }
        else
        {
            return target.position - agent.position;
        }

        //return result;
    }

    private float MapOrientation(float i)
    {
        //we will map the value to (-pi, pi] interval
        if (i <= Mathf.PI && i > -Mathf.PI)
        {
            return i;
        }
        else if (i > Mathf.PI)
        {
            while (i > Mathf.PI)
            {
                i -= 2 * Mathf.PI;
            }
            return i;
        }
        else
        {
            while (i <= -Mathf.PI)
            {
                i += 2 * Mathf.PI;
            }
            return i;
        }
    }

    //function used to seek to a position with max acc
    public Vector3 SeekPosWithMaximumAcc(Vector3 pos)
    {
        Vector3 steering = new Vector3();

        //get direction to player
        steering = pos - agent.position;
        steering = steering.normalized * maxAcceleration;
        //agent.DrawCircle(steering + agent.position, 1.0f);
        return steering;
    }

    public Vector3 DynamicArrive()
    {
        //get the distance between target and agent
        Vector3 direction = GetDirectionVec();
        float distance = GetDistance();
        //visual effects for target
        target.DrawConcentricCircle(slowRadiusL);
        //distance check
        if (distance < slowRadiusL && distance > targetRadiusL)
        {
            //here is the condition we need to think about reduce speed
            float temp = distance - targetRadiusL;
            float targetSpeed = maxSpeed * (temp / (slowRadiusL - targetRadiusL));
            Vector3 targetVelocity = direction.normalized * targetSpeed;
            Vector3 linear = targetVelocity - agent.velocity;
            linear /= timeToTarget;
            /*
             * Note: since linear /= timeToTarget will get the acceleration
             * we need to perform per time, we need to devide the linear with
             * another deltaTime in order to make sure the enough acceleration
             * is applied to the gameobject
             */
            linear /= Time.deltaTime;
            return linear;
        }

        return new Vector3(0, 0, 0);
    }

    public float Align(Vector3 targetVec)
    {
        //get direction to target
        Vector3 direction = targetVec - agent.position;
        float rotation = Mathf.Atan2(direction.x, direction.z) - agent.orientation;
        float targetRotation, steering, rotationDirection;

        //map result to (-pi, pi) interval
        rotation = MapOrientation(rotation);
        float rotationSize = Mathf.Abs(rotation);

        //check if we are there
        if (rotationSize < targetRadiusA) { return 0f; }

        //if oiutside slow radius use max rotation
        if (rotationSize > slowRadiusA) { targetRotation = maxRotation; }

        //otherwise use scaled rotation
        else { targetRotation = maxRotation * (rotationSize / slowRadiusL); }

        //final target rotation use speed and direction
        targetRotation *= rotation / rotationSize;

        //acceleration tries to get to the target rotation
        steering = targetRotation - agent.rotation;

        //check if acceleration is too high
        float angularAccel = Mathf.Abs(steering);
        if (angularAccel > maxAngularAcceleration)
        {
            steering /= angularAccel;
            steering *= maxAngularAcceleration;
        }
        //print("Steering is: " + steering);
        //agent.DrawFaceCircle(1.0f, 5.0f);
        return steering;
    }

    //this fastAlign is used for wall avoidence since 
    //we need a greater acceleration to stop the player
    public float FastAlign(Vector3 targetVec)
    {
        //get direction to target
        Vector3 direction = targetVec - agent.position;
        float rotation = Mathf.Atan2(direction.x, direction.z) - agent.orientation;
        float targetRotation, steering, rotationDirection;

        //map result to (-pi, pi) interval
        rotation = MapOrientation(rotation);
        float rotationSize = Mathf.Abs(rotation);

        //check if we are there
        if (rotationSize < 0.01f) { return 0f; }

        //if oiutside slow radius use max rotation
        if (rotationSize > 0.2f) { targetRotation = 10f; }

        //otherwise use scaled rotation
        else { targetRotation = 10f * (rotationSize / 0.2f); }

        //final target rotation use speed and direction
        targetRotation *= rotation / rotationSize;

        //acceleration tries to get to the target rotation
        steering = targetRotation - agent.rotation;

        //check if acceleration is too high
        float angularAccel = Mathf.Abs(steering);
        if (angularAccel > 10f)
        {
            steering /= angularAccel;
            steering *= 10f;
        }
        //print("Steering is: " + steering);
        //agent.DrawFaceCircle(1.0f, 5.0f);
        return steering;
    }

    public wanderSteering Wander()
    {
        if (lrrhWander)
        {
            if ((agent.position - wanderTarget.position).magnitude <= targetRadiusL)
            {
                wanderTarget = Path[Random.Range(0, Path.Length)].transform;
            }
        }

        //update orientation
        wanderOrientation += (Random.value - Random.value) * wanderRate;

        //get target orientation
        float targetOr = agent.orientation + wanderOrientation;

        //get center of wander cirlce
        Vector3 position = agent.position + wanderOffset * new Vector3(Mathf.Sin(agent.orientation), 0, Mathf.Cos(agent.orientation));

        position += wanderRadius * new Vector3(Mathf.Sin(targetOr), 0, Mathf.Cos(targetOr));
        //print("Target location is: " + position);

        //create struct to hold output
        wanderSteering ret;

        //check for wall avoidance otherwise delegate angular to face
        wallAvoidanceSteering wallAvoidance = Wallavoidance();
        if (System.Math.Abs(wallAvoidance.angular) < Mathf.Epsilon)
        {
            //change wanerTarget so its not set on the other NPC
            if (lrrhWander)
            {
                ret.angular = Align(wanderTarget.position);
            }
            //else keep normal wander functionality
            else
            {
                ret.angular = Align(target.position);
            }

        }
        else
        {
            ret.angular = wallAvoidance.angular;
        }

        //check for collision detection otherwise keep linear normal

        Vector3 foundCollision = CollisionDetectAndAvoid();
        if (foundCollision == Vector3.zero)
        {
            //change Wander Target so its not set on the other NPC
            if (lrrhWander)
            {
                //check if you should dynamically arrive
                if ((wanderTarget.position - agent.position).magnitude < slowRadiusL)
                {
                    ret.linear = DynamicArrive();
                    return ret;
                }
                //else keep max acceleration
                else
                {
                    ret.linear = maxAcceleration * new Vector3(Mathf.Sin(agent.orientation), 0, Mathf.Cos(agent.orientation));
                    return ret;
                }
            }
            //else keep normal functionality
            else
            {
                //check if you should dynamically arrive
                if ((target.position - agent.position).magnitude < slowRadiusL)
                {
                    ret.linear = DynamicArrive();
                    return ret;
                }
                //else keep max acceleration
                else
                {
                    ret.linear = maxAcceleration * new Vector3(Mathf.Sin(agent.orientation), 0, Mathf.Cos(agent.orientation));
                    return ret;
                }
            }
        }
        else
        {
            ret.linear = foundCollision;
            return ret;
        }


    }

    //this is the wall avoidance function to help NPC avoid the collision
    public wallAvoidanceSteering Wallavoidance()
    {
        //the variables to store raycast result
        RaycastHit hit;
        RaycastHit hit2;
        wallAvoidanceSteering result;
        Vector3 faceVec = Quaternion.Euler(0.0f, agent.rotation, 0.0f) * agent.transform.forward;
        Debug.DrawRay(agent.position, faceVec * raycastDistance, Color.red, Time.deltaTime);


        //do the sphereCast which will cast a sphere instead of a line
        if (Physics.SphereCast(agent.position, 0.85f, faceVec, out hit, raycastDistance))
        {
            //print(hit.collider.gameObject.name);

            if (hit.collider.gameObject.tag == "Obstacle")
            {
                //draw the raycast and seek the avoidence position
                Debug.DrawRay(hit.point, hit.normal * raycastDistance, Color.blue, Time.deltaTime);

                //calculate the acceleration vector
                Vector3 targetPos = hit.point + hit.normal * raycastDistance;
                result.linear = SeekPosWithMaximumAcc(targetPos);

                //if it is too close, we need a acceleration greater than 
                //maxAcceleration to decrease the speed greatly
                if (hit.distance < 2.25f)
                {
                    float multipler = 1f / hit.distance;
                    result.linear *= multipler;

                    //if the acceleration is too large, we need to limit the max acceleration
                    //in order to avoid the bounce effect which could happen if the player is 
                    //too close to the obstacle

                    if (result.linear.magnitude > 20.0f * maxAcceleration)
                    {
                        result.linear = result.linear.normalized * 20.0f * maxAcceleration;
                    }

                }

                //then we do the turing, we will check 45, 90, ... , 180
                //if all these failed, we will use targetPos
                float ang = 0f;
                while (ang < 360f)
                {
                    //check collision using simple ray cast
                    bool cannotTurn = Physics.Raycast(agent.position, Quaternion.Euler(0.0f, agent.rotation + ang, 0.0f) * agent.transform.forward, out hit2, raycastDistance);
                    //check the tag of collision object, the only situation
                    //cannot turn is find a obstacle tagged gameobject
                    if (cannotTurn == true)
                    {
                        if (hit2.collider.gameObject.tag != "Obstacle")
                        {
                            cannotTurn = false;
                        }
                    }

                    //if there is no obstacle, just turn the gameObject
                    if (cannotTurn == false)
                    {
                        //turn 45 degrees every time in clockwise
                        targetPos = agent.position + Quaternion.Euler(0.0f, agent.rotation + ang, 0.0f) * agent.transform.forward * raycastDistance;

                        //if the target is too close to the obstacle, turn faster
                        if (hit.distance < 0.45f)
                        {
                            result.angular = FastAlign(targetPos);
                        }
                        else
                        {
                            result.angular = Align(targetPos);
                        }
                        result.needAvoidance = true;
                        return result;
                    }
                    ang += 45f;

                    //make sure turn back is not selected since
                    //hit.normal could be looking back
                    if (ang - 180f <= 0.01f)
                    {
                        continue;
                    }
                }

                //if the all the previous checks are not passed, just
                //turn to the direction of normal position
                targetPos = hit.point + hit.normal * raycastDistance;
                if (hit.distance < 1.0f)
                {
                    result.angular = FastAlign(targetPos);
                }
                else
                {
                    result.angular = Align(targetPos);
                }
                result.needAvoidance = true;
                return result;
            }
        }


        result.linear = new Vector3(0f, 0f, 0f);
        result.angular = 0f;
        result.needAvoidance = false;
        return result;

    }

    //This function is used for collision detection and avoidence among characters
    public Vector3 CollisionDetectAndAvoid()
    {
        //print("Are we even checking?");
        List<GameObject> allNPCs;
        //error checking: if not find the script attached on Phase Manager
        if (GameObject.Find("PhaseManager") == null)
        {
            return new Vector3(0f, 0f, 0f);
        }
        else if (GameObject.Find("PhaseManager").GetComponent<FieldMapManager>() == null)
        {
            return new Vector3(0f, 0f, 0f);
        }

        //get a list for all NPCs
        if(this.name[0] == 'R')
        {
            enemies = GameObject.Find("PhaseManager").GetComponent<FieldMapManager>().spawnedWolfs;
        }
        else
        {
            enemies = GameObject.Find("PhaseManager").GetComponent<FieldMapManager>().spawnedReds;
        }


        //if there is only one NPC
        if (enemies.Count <= 1)
        {
            return new Vector3(0f, 0f, 0f);
        }

        //if there are multiple NPCs
        GameObject nearestNPC = null;
        float shortestTime = 99999999f;
        float firstMinSeperation = 0;
        float firstDistance = 0;
        Vector3 firstRelativePos = new Vector3(0f, 0f, 0f);
        Vector3 firstRelativeVel = new Vector3(0f, 0f, 0f);
        foreach (GameObject NPC in enemies)
        {
            //we should not check self
            if (this.name != NPC.name)
            {
                //calculate the time to collision
                NPCController nc = NPC.GetComponent<NPCController>();
                Vector3 relativePos = nc.position - agent.position;
                Vector3 relativeVel = nc.velocity - agent.velocity;
                float relativeSpeed = relativeVel.magnitude;
                float timeToCollision = -Vector3.Dot(relativePos, relativeVel) / (relativeSpeed * relativeSpeed);

                //check collision
                float distance = relativePos.magnitude;
                float minSeperation = distance - relativeSpeed * shortestTime;
                if (minSeperation > 2 * targetRadiusL)
                {
                    continue;
                }

                //check if is already in collision
                if (distance < 2 * targetRadiusL)
                {
                    return (relativePos.normalized * (-maxAcceleration) * 2.0f);
                }

                //check if the collisiontime is the shortest
                if (timeToCollision > 0 && timeToCollision < shortestTime)
                {
                    shortestTime = timeToCollision;
                    nearestNPC = NPC;
                    firstDistance = distance;
                    firstMinSeperation = minSeperation;
                    firstRelativePos = relativePos;
                    firstRelativeVel = relativeVel;
                }
            }
        }

        //if we need to do the avoidance   
        if (nearestNPC != null)
        {
            NPCController nc = nearestNPC.GetComponent<NPCController>();
            Vector3 relativePos;

            //if we are exactly going to collide or already collided
            if (firstMinSeperation <= 0 || (nc.position - agent.position).magnitude < 2 * targetRadiusL)
            {
                relativePos = nc.position - agent.position;
            }
            //otherwise we calculate the future relative position
            else
            {
                relativePos = firstRelativePos + firstRelativeVel * shortestTime;
            }

            //do the avoidance
            relativePos = relativePos.normalized;
            return (relativePos * maxAcceleration * 1.2f);
        }

        return new Vector3(0f, 0f, 0f);
    }

    public wanderSteering ConeCheck()
    {
        //get a list for all NPCs
        List<GameObject>  allNPCs = GameObject.Find("PhaseManager").GetComponent<FieldMapManager>().AllNPCs();


        wanderSteering emptyRet;
        emptyRet.linear = Vector3.zero;
        emptyRet.angular = 0f;

        //if there is only one NPC
        if (allNPCs.Count <= 1)
        {
            return emptyRet;
        }
        foreach (GameObject NPC in allNPCs)
        {
            if(this.name != NPC.name)
            {
                Vector3 orientationAsVec = new Vector3(Mathf.Sin(agent.orientation), 0, Mathf.Cos(agent.orientation));
                Vector3 direction = NPC.GetComponent<NPCController>().position - agent.position;
                if (Vector3.Dot(orientationAsVec, direction) > Mathf.Cos(Mathf.Deg2Rad * (coneThreshold/2)) && direction.magnitude < targetRadiusL*3)
                {
                    this.target = NPC.GetComponent<NPCController>();
                    print(this.name + " is evading");
                    return DynamicEvade();
                }
                else
                {
                    return emptyRet;
                }
            }
        }

        return emptyRet;

    }

}
