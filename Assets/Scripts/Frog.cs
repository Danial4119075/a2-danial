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

    // ---- A* Path Following State ----
    // The sequence of grid nodes that make up the current A* path (start → target).
    // Computed once per right-click; not recalculated every frame.
    private Node[] _path;

    // Index into _path of the waypoint the frog is currently steering towards.
    private int _pathIndex;

    // True while there is an active A* path the frog should be following.
    private bool _hasPath;

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

            // Set the arrival radius dynamically based on click distance.
            // The frog slows down proportionally to how close it is to the target,
            // preventing overshoot. The radius is clamped to sensible min/max values.
            _arriveRadius = Mathf.Clamp(
                ArrivePct * ((Vector2)_lastClickPos - (Vector2)transform.position).magnitude,
                MinArriveRadius,
                MaxArriveRadius);

            // Move the flag sprite to the clicked position so the player can see the target.
            _flag.position = (Vector2)_lastClickPos + new Vector2(0.55f, 0.55f);
            _flagSr.enabled = true;

            // Request a new A* path from the frog's current position to the click point.
            // We compute the path ONCE per click — not every frame — to avoid the
            // "needlessly high computational burden" mentioned in the assignment spec.
            // The frog then follows the pre-computed waypoints until it arrives.
            _path      = Pathfinding.RequestPath(transform.position, (Vector2)_lastClickPos);
            _pathIndex = 0;       // start from the first waypoint in the new path
            _hasPath   = (_path != null && _path.Length > 0);
        }
        else // show debug lines for nearest fly, snake, and the current A* path
        {
            if (closestFly != null)
                Debug.DrawLine(transform.position, closestFly.transform.position, Color.black);
            if (closestSnake != null)
                Debug.DrawLine(transform.position, closestSnake.transform.position, Color.red);

            // Draw the remaining A* path in the Scene view so it's visible during the demo.
            // Yellow = line from frog to the current waypoint it's heading towards.
            // Green  = lines between the remaining waypoints ahead.
            if (_hasPath && _path != null)
            {
                if (_pathIndex < _path.Length)
                    Debug.DrawLine(transform.position, _path[_pathIndex].worldPosition, Color.yellow);

                for (int i = _pathIndex; i < _path.Length - 1; i++)
                    Debug.DrawLine(_path[i].worldPosition, _path[i + 1].worldPosition, Color.green);
            }
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

    //TODO Implement the following Decision Tree (Task — not yet implemented)
    // no health <= 0 --> set speed to 0 and color to red (1, 0.2, 0.2)
    // user clicked --> go to that click
    // nearby/outside of screen --> go towards screen (similar to flies)
    // closest snake nearby --> flee from snake within the screen
    // closest fly within screen --> go towards that fly
    // otherwise --> go to center of the screen

    // Decides what velocity the frog should move at this physics step.
    // Currently: follow the A* path if one exists, else stay still.
    // (The Decision Tree will extend this in a later task.)
    private Vector2 decideMovement()
    {
        // If we have an active A* path, follow it waypoint by waypoint.
        if (_hasPath)
        {
            return followAStarPath();
        }

        // No target — stay still.
        return Vector2.zero;
    }

    // Steers the frog along its pre-computed A* path, one waypoint at a time.
    //
    // Strategy (as specified):
    //   - SEEK toward every intermediate waypoint  → full speed, no deceleration.
    //   - ARRIVE at the final waypoint             → decelerate to stop naturally.
    //
    // Why Seek for intermediates?
    //   Arrive would slow the frog down before each grid waypoint, making movement
    //   look choppy and sluggish. Seek keeps full speed through the path and only
    //   decelerates at the actual destination, which looks much more natural.
    //
    // Waypoint advancement:
    //   When the frog comes within TARGET_REACHED_TOLERANCE of the current waypoint,
    //   it increments _pathIndex to target the next one. This avoids recomputing
    //   A* every frame (which the spec explicitly says to avoid).
    private Vector2 followAStarPath()
    {
        // Guard: no path or already finished it.
        if (_path == null || _pathIndex >= _path.Length)
        {
            _hasPath      = false;
            _lastClickPos = null;
            if (HideFlagOnceReached) _flagSr.enabled = false;
            return Vector2.zero;
        }

        // World position of the waypoint we are currently heading towards.
        Vector2 waypointPos = _path[_pathIndex].worldPosition;

        // Check if we have arrived within tolerance of the current waypoint.
        float distToWaypoint = ((Vector2)transform.position - waypointPos).magnitude;
        if (distToWaypoint <= Constants.TARGET_REACHED_TOLERANCE)
        {
            _pathIndex++; // advance to the next waypoint in the path

            // If that was the last waypoint, the journey is complete.
            if (_pathIndex >= _path.Length)
            {
                _hasPath      = false;
                _lastClickPos = null;
                if (HideFlagOnceReached) _flagSr.enabled = false;
                return Vector2.zero;
            }

            // Update waypointPos to the freshly targeted node.
            waypointPos = _path[_pathIndex].worldPosition;
        }

        bool isLastWaypoint = (_pathIndex == _path.Length - 1);

        if (isLastWaypoint)
        {
            // ARRIVE at the final destination: speed scales down with distance
            // inside _arriveRadius so the frog decelerates and stops precisely.
            return Steering.ArriveDirect(transform.position, waypointPos, _arriveRadius, MaxSpeed);
        }
        else
        {
            // SEEK toward the next intermediate waypoint at full speed.
            return Steering.SeekDirect(transform.position, waypointPos, MaxSpeed);
        }
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
