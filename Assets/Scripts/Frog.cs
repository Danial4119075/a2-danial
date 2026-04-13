using UnityEngine;
using UnityEngine.InputSystem;
using SteeringCalcs;
using Globals;

public class Frog : MonoBehaviour
{
    // Frog status.
    public int Health;

    // Steering parameters.
    public float MaxSpeed;
    public float MaxAccel;
    public float AccelTime;

    // The arrival radius is set up to be dynamic, depending on how far away
    // the player right-clicks from the frog. See the logic in Update().
    public float ArrivePct;
    public float MinArriveRadius;
    public float MaxArriveRadius;
    private float _arriveRadius;

    // Turn this off to make it easier to see overshooting when seek is used
    // instead of arrive.
    public bool HideFlagOnceReached;

    // References to various objects in the scene that we want to be able to modify.
    private Transform _flag;
    private SpriteRenderer _flagSr;
    private DrawGUI _drawGUIScript;
    private Animator _animator;
    private Rigidbody2D _rb;
    private InputAction ClickMoveAction;
    // Stores the last position that the player right-clicked. Initially null.
    private Vector2? _lastClickPos;

    //WEEK6
    //Used by DTs to make decision
    public float scaredRange;
    public float huntRange;
    private Fly closestFly;
    private Snake closestSnake;
    private float distanceToClosestFly;
    private float distanceToClosestSnake;
    public float anchorWeight;
    public Vector2 AnchorDims;



    void Start()
    {
        // Initialise the various object references.
        _flag = GameObject.Find("Flag").transform;
        _flagSr = _flag.GetComponent<SpriteRenderer>();
        _flagSr.enabled = false;

        GameObject uiManager = GameObject.Find("UIManager");
        if (uiManager != null)
        {
            _drawGUIScript = uiManager.GetComponent<DrawGUI>();
        }

        _animator = GetComponent<Animator>();

        _rb = GetComponent<Rigidbody2D>();
        ClickMoveAction = InputSystem.actions.FindAction("Attack");

        _lastClickPos = null;
        _arriveRadius = MinArriveRadius;

    }

    void Update()
    {

        // Check if the player right-clicked (mouse button #1).
        if (ClickMoveAction.WasPressedThisFrame())
        {
            _lastClickPos = Camera.main.ScreenToWorldPoint(Mouse.current.position.ReadValue());

            // Set the arrival radius dynamically.
            _arriveRadius = Mathf.Clamp(ArrivePct * ((Vector2)_lastClickPos - (Vector2)transform.position).magnitude, MinArriveRadius, MaxArriveRadius);

            _flag.position = (Vector2)_lastClickPos + new Vector2(0.55f, 0.55f);
            _flagSr.enabled = true;
        }
        else // show the relevant info about fly and snake
        {
            if (closestFly != null)
                Debug.DrawLine(transform.position, closestFly.transform.position, Color.black);
            if (closestSnake != null)
                Debug.DrawLine(transform.position, closestSnake.transform.position, Color.red);
        }

    }

    void FixedUpdate()
    {

        Vector2 desiredVel = decideMovement();
        Debug.DrawLine((Vector2)transform.position, (Vector2)transform.position + desiredVel, Color.blue);
        Vector2 steering = Steering.DesiredVelToForce(desiredVel, _rb, AccelTime, MaxAccel);
        _rb.AddForce(steering);

        UpdateAppearance();
    }

    private void UpdateAppearance()
    {
        if (_rb.linearVelocity.magnitude > Constants.MIN_SPEED_TO_ANIMATE)
        {
            _animator.SetBool("Walking", true);
            transform.up = _rb.linearVelocity;
        }
        else
        {
            _animator.SetBool("Walking", false);
        }
    }

    public void TakeDamage()
    {
        if (Health > 0)
        {
            Health--;
        }
    }

    //TODO Implement the following Decision Tree
    // no health <= 0 --> set speed to 0 and color to red (1, 0.2, 0.2)
    // user clicked --> go to that click
    // nearby/outside of screen --> go towards screen (similar to flies)
    // closest snake nearby --> flee from snake within the screen
    // closest fly within screen --> go towards that fly
    // otherwise --> go to center of the screen

    //TODO SUGGESTED IMPROVEMENTS:
    //go to the center of mass of flies within screen
    //if 2 snake nearby -> freeze
    //Handle shooting bubbles
    //Come up with a better DT, for example: find flies that are within a circle around the frog that doesnt include any snake
    //Extra0 shoot bubble?
    //Extra1 update your code so that: 
    //Extra2 update your code with a better DT (find flies that are within a circle around the frog that doesnt include any snake)
    //Gameplay: tweak speed, range, acceleration and anchoring
    private Vector2 decideMovement()
    {
        if (_lastClickPos != null)
        {
            return (getVelocityTowardsFlag());
        }

        else
        {
            return (Vector2.zero);
        }
    }

    private Vector2 getVelocityTowardsFlag()
    {
        Vector2 desiredVel = Vector2.zero;
        if (_lastClickPos != null)
        {
            if (((Vector2)_lastClickPos - (Vector2)gameObject.transform.position).magnitude > Constants.TARGET_REACHED_TOLERANCE)
            {
                desiredVel = Steering.ArriveDirect(gameObject.transform.position, (Vector2)_lastClickPos, _arriveRadius, MaxSpeed);
            }
            else
            {
                _lastClickPos = null;

                if (HideFlagOnceReached)
                {
                    _flagSr.enabled = false;
                }
            }

        }
        return desiredVel;
    }

    private void findClosestFly()
    {
        distanceToClosestFly = Mathf.Infinity;

        foreach (Fly fly in (Fly[])GameObject.FindObjectsByType(typeof(Fly), FindObjectsSortMode.None))
        {
            float distanceToFly = (fly.transform.position - transform.position).magnitude;
            if (fly.GetComponent<Fly>().State != Fly.FlyState.Dead)
            {
                if (distanceToFly < distanceToClosestFly)
                {
                    closestFly = fly;
                    distanceToClosestFly = distanceToFly;

                }
            }

        }
    }

    //TODO See findClosestFly for inspiration
    private void findClosestSnake()
    {

    }

    //TODO Check wether the current transform is out of screen (true) or not (false)
    private bool isOutOfScreen(Transform transform)
    {
        return false;
    }

}
