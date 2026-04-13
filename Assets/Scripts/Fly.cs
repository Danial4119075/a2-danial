using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using SteeringCalcs;

public class Fly : MonoBehaviour
{
    // Current FSM State, tracked publically so we can modify this (live) in Unity editor
    public FlyState State;

    // Parameters controlling transitions in/out of the Flee state.
    public float StopFleeingRange;
    public float FrogStillFleeRange;
    public float FrogMovingFleeRange;
    public float FrogAlertSpeed;
    public float BubbleFleeRange;

    // Time taken to respawn after being eaten by the frog.
    public float RespawnTime;

    // References to various objects in the scene that we want to be able to modify.
    private Rigidbody2D _rb;
    private SpriteRenderer _sr;
    private Transform _frog;
    private Rigidbody2D _frogRb;

    // Reference to the flocking parameters (attached to the "Flock"
    // game object in the "FlockingTest" and "FullGame" scenes).
    private FlockSettings _settings;

    // FSM tracking properties
    // Time since eaten by the frog (0 when alive).
    private float _timeDead;

    // List tracking current neighbours of this flut
    List<Transform> _neighbours;

    // Fly FSM states
    public enum FlyState : int
    {
        Flocking = 0,
        Alone = 1,
        Fleeing = 2,
        Dead = 3,
        Respawn = 4
    }

    // Fly FSM events
    public enum FlyEvent : int
    {
        JoinedFlock = 0,
        LostFlock = 1,
        ScaredByFrog = 2,
        EscapedFrog = 3,
        CaughtByFrog = 4,
        RespawnTimeElapsed = 5,
        NowAlive = 6,

        // NOTE: Remove this state for the Fly
        ScaredByBubble = 7
    }

    void Start()
    {
        _settings = transform.parent.GetComponent<FlockSettings>();
        _rb = GetComponent<Rigidbody2D>();
        _sr = GetComponent<SpriteRenderer>();

        // Have to be a bit careful setting _frog, since the frog doesn't exist in all scenes.
        GameObject frog = GameObject.Find("Frog");
        if (frog != null)
        {
            _frog = frog.transform;
            _frogRb = frog.GetComponent<Rigidbody2D>();
        }

        // Initial FSM variables
        _timeDead = 0.0f;
        _neighbours = new List<Transform>();
    }

    void FixedUpdate()
    {
        // Update data needed in computing FSM transitions & states
        UpdateNeighbours();

        // Events triggered by each fixed update tick
        FixedUpdateEvents();

        // Update the Fly behaviour based on the current FSM state
        FSM_State();

        // Configure final appearance
        UpdateAppearance();
    }

    // Trigger Events for each fixed update tick, using an event first FSM implementation
    void FixedUpdateEvents()
    {

        // If the fly's been dead long enough, trigger respawn event
        if (State == FlyState.Dead)
        {
            _timeDead += Time.fixedDeltaTime;
            if (_timeDead > RespawnTime)
            {
                HandleEvent(FlyEvent.RespawnTimeElapsed);
            }
        }

        // Check triggers from flocking events
        // Note: These can be true each update but may not trigger a state transition.
        if (State == FlyState.Flocking ||
            State == FlyState.Fleeing ||
            State == FlyState.Alone)
        {
            if (_neighbours.Count == 0)
            {
                HandleEvent(FlyEvent.LostFlock);
            }
            else
            {
                HandleEvent(FlyEvent.JoinedFlock);
            }
        }

        // Check triggers from the frog and bubbles
        if (_frog != null)
        {
            // Check if we've been scared by the frog.
            if (_frogRb.linearVelocity.magnitude >= FrogAlertSpeed && ((transform.position - _frog.transform.position).magnitude < FrogMovingFleeRange)
                || _frogRb.linearVelocity.magnitude < FrogAlertSpeed && ((transform.position - _frog.transform.position).magnitude < FrogStillFleeRange))
            {
                HandleEvent(FlyEvent.ScaredByFrog);
            }

            // Check if we've been scared by a bubble.
            GameObject[] bubbles = GameObject.FindGameObjectsWithTag("Bubble");
            foreach (GameObject bubble in bubbles)
            {
                if ((transform.position - bubble.transform.position).magnitude < BubbleFleeRange)
                {
                    HandleEvent(FlyEvent.ScaredByBubble);
                    break;
                }
            }

            // Check if we no longer need to be scared (the frog isn't close and no bubbles are close)
            bool escaped = true;
            if ((transform.position - _frog.transform.position).magnitude <= StopFleeingRange)
            {
                escaped = false;

            }
            else
            {
                foreach (GameObject bubble in bubbles)
                {
                    if ((transform.position - bubble.transform.position).magnitude <= StopFleeingRange)
                    {
                        escaped = false;
                        break;
                    }
                }
            }

            if (escaped)
            {
                HandleEvent(FlyEvent.EscapedFrog);
            }
        }
    }

    // Process the current FSM state, using an event first FSM implementation
    private void FSM_State()
    {
        // Common variables between states
        Vector2 desiredVel = Vector2.zero;

        if (State == FlyState.Dead)
        {
            // UpdateAppearance ensures the sprite render is off
            // Ensure dead fly stops moving
            desiredVel = Vector2.zero;
        }
        else if (State == FlyState.Respawn)
        {
            // Note this causes an immediate FSM transition
            Respawn();

            // Ensure initial zero velocity
            desiredVel = Vector2.zero;
        }
        else if (State == FlyState.Flocking)
        {
            Vector2 desiredSep = _settings.SeparationWeight * Steering.GetSeparation(transform.position, _neighbours, _settings.MaxSpeed);
            Vector2 desiredCoh = _settings.CohesionWeight * Steering.GetCohesion(transform.position, _neighbours, _settings.MaxSpeed);
            Vector2 desiredAli = _settings.AlignmentWeight * Steering.GetAlignment(_neighbours, _settings.MaxSpeed);
            Vector2 desiredAnch = _settings.AnchorWeight * Steering.GetAnchor(transform.position, _settings.AnchorDims);

            // Draw the forces for debugging purposes.
            //Debug.DrawLine(transform.position, (Vector2)transform.position + desiredSep, Color.red);
            //Debug.DrawLine(transform.position, (Vector2)transform.position + desiredCoh, Color.green);
            //Debug.DrawLine(transform.position, (Vector2)transform.position + desiredAli, Color.blue);
            //Debug.DrawLine(transform.position, (Vector2)transform.position + desiredAnch, Color.yellow);

            desiredVel = (desiredSep + desiredCoh + desiredAli + desiredAnch).normalized * _settings.MaxSpeed;
        }
        else if (State == FlyState.Alone)
        {
            Transform nearestFly = null;

            foreach (Transform flockMember in transform.parent)
            {
                if (flockMember.GetComponent<Fly>().State != FlyState.Dead && flockMember != transform)
                {
                    if (nearestFly == null || (transform.position - flockMember.position).magnitude < (transform.position - nearestFly.position).magnitude)
                    {
                        nearestFly = flockMember;
                    }
                }
            }

            if (nearestFly != null)
            {
                desiredVel = Steering.SeekDirect(transform.position, nearestFly.position, _settings.MaxSpeed);
                Debug.DrawLine(transform.position, nearestFly.position, Color.yellow);
            }
        }
        else if (State == FlyState.Fleeing)
        {
            desiredVel = Steering.FleeDirect(transform.position, _frog.position, _settings.MaxSpeed);
        }

        // Convert the desired velocity to a force, then apply it.
        // All states use this
        Vector2 steering = Steering.DesiredVelToForce(desiredVel, _rb, _settings.AccelTime, _settings.MaxAccel);
        _rb.AddForce(steering);
    }

    private void Respawn()
    {
        // Respawn 20 units away from the origin at a random angle.
        // The flocking forces should automatically bring the fly back into the main arena.
        float randomAngle = Random.Range(-Mathf.PI, Mathf.PI);
        Vector2 randomDirection = new Vector2(Mathf.Cos(randomAngle), Mathf.Sin(randomAngle));
        transform.position = 20.0f * randomDirection;

        _timeDead = 0.0f;

        // Immediately trigger Respawn trigger
        HandleEvent(FlyEvent.NowAlive);
    }

    // UpdateNeighbours() sets all flock members that:
    // - Are not dead.
    // - Are not equal to this flock member.
    // - Are within a distance of _settings.FlockRadius from this flock member.
    private void UpdateNeighbours()
    {
        _neighbours.Clear();

        foreach (Transform flockMember in transform.parent)
        {
            if (flockMember.GetComponent<Fly>().State != FlyState.Dead
                && flockMember != transform && (transform.position - flockMember.position).magnitude < _settings.FlockRadius)
            {
                _neighbours.Add(flockMember);
            }
        }
    }

    // Set the Fly appearance based on it's movement and FSM state
    private void UpdateAppearance()
    {
        _sr.flipX = _rb.linearVelocity.x > 0;

        // Update color to provide a visual indication of the current state
        if (State == FlyState.Flocking)
        {
            _sr.enabled = true;
            _sr.color = new Color(1.0f, 1.0f, 1.0f);
        }
        else if (State == FlyState.Alone)
        {
            _sr.enabled = true;
            _sr.color = new Color(1.0f, 0.52f, 0.01f);
        }
        else if (State == FlyState.Fleeing)
        {
            _sr.enabled = true;
            _sr.color = new Color(0.45f, 0.98f, 0.94f);
        }
        else if (State == FlyState.Dead)
        {
            _sr.enabled = false;
        }
        else if (State == FlyState.Respawn)
        {
            _sr.enabled = false;
        }
    }

    private void OnTriggerEnter2D(Collider2D collider)
    {
        if (collider.gameObject.tag.Equals("Frog"))
        {
            // Transition FSM
            HandleEvent(FlyEvent.CaughtByFrog);

            // Also update Frog
            //collider.gameObject.GetComponent<Frog>().CatchFly();
        }
    }

    private void SetState(FlyState newState)
    {
        if (newState != State)
        {
            // Can uncomment this for debugging purposes.
            // Debug.Log(name + " switching state to " + newState.ToString());

            State = newState;
        }
    }

    // HandleEvent implements the transition logic of the FSM.
    //      This can be called with invalid transitions - so check first!
    private void HandleEvent(FlyEvent e)
    {
        //Debug.Log(name + " handling event " + e.ToString());

        // FSM Hierarchy
        if (State == FlyState.Dead)
        {
            if (e == FlyEvent.RespawnTimeElapsed)
            {
                SetState(FlyState.Respawn);
            }
        }
        else if (State == FlyState.Respawn)
        {
            if (e == FlyEvent.NowAlive)
            {
                SetState(FlyState.Flocking);
            }
        }
        // Second Hierarchy Layer
        else
        {
            // All can transition to Dead
            if (e == FlyEvent.CaughtByFrog)
            {
                SetState(FlyState.Dead);
            }

            // Otherwise cheack each state transition
            else if (State == FlyState.Flocking)
            {
                if (e == FlyEvent.LostFlock)
                {
                    SetState(FlyState.Alone);
                }
                else if (e == FlyEvent.ScaredByFrog || e == FlyEvent.ScaredByBubble)
                {
                    SetState(FlyState.Fleeing);
                }
            }
            else if (State == FlyState.Alone)
            {
                if (e == FlyEvent.JoinedFlock)
                {
                    SetState(FlyState.Flocking);
                }
                else if (e == FlyEvent.ScaredByFrog || e == FlyEvent.ScaredByBubble)
                {
                    SetState(FlyState.Fleeing);
                }
            }
            else if (State == FlyState.Fleeing)
            {
                if (e == FlyEvent.EscapedFrog)
                {
                    SetState(FlyState.Flocking);
                }

            }
        }
    }
}
