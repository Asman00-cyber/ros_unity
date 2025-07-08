using Unity.VisualScripting;
using UnityEditor.PackageManager.UI;
using UnityEngine;
using static UnityEditor.ShaderData;
using UnityEngine.XR;

[RequireComponent(typeof(CharacterController))]
public class FreeLookMover : MonoBehaviour
{
    [Header("Movement")]
    public float moveSpeed = 5f;   // m/s
    public float acceleration = 10f;  // lerp factor

    [Header("Mouse look")]
    public float mouseSensitivity = 2f;  // deg per pixel
    public float pitchClamp = 80f; // up/down limit

    CharacterController controller;
    Vector3 velocity;
    float yaw, pitch;

    void Awake()
    {
        controller = GetComponent<CharacterController>();

        Cursor.lockState = CursorLockMode.None;   // never lock
        Cursor.visible = true;                  // always show
    }

    void Update()
    {
        HandleMouseLook();
        HandleMovement();
    }

    /* Camera rotation – only while RIGHT mouse is held */
    void HandleMouseLook()
    {
        if (Input.GetMouseButton(1))              // RMB
        {
            yaw += Input.GetAxis("Mouse X") * mouseSensitivity;
            pitch -= Input.GetAxis("Mouse Y") * mouseSensitivity;
            pitch = Mathf.Clamp(pitch, -pitchClamp, pitchClamp);

            transform.localRotation = Quaternion.Euler(pitch, yaw, 0f);
        }
    }
    // Input.GetAxis("Mouse X") returns how many pixels the mouse moved this frame along the X screen-axis (left = −, right = +).
    //1)Looks at the proposed new pitch after you moved the mouse.
    //2) Keeps it if it’s within the safe window[-pitchClamp … +pitchClamp].
    //3) Otherwise snaps it to the nearest edge of that window.
    //4) With pitchClamp = 80 you can tilt the camera up to 80 ° above the horizon and down to 80 ° below, but never flip over or spin past vertical.

    /* WASD movement relative to camera forward */
    // Build movement vector from WASD (A/D = X, W/S = Z).
    // Example: W+D → (1,0,1) whose length is √2 ≈ 1.41.
    // input.Normalize() scales it to length 1 (0.707,0,0.707) so
    // diagonal speed isn’t 41 % faster than straight‑ahead.
    // TransformDirection turns this local‑space vector into world‑space,
    // then * moveSpeed gives a velocity of exactly moveSpeed m/s in
    // whichever direction the camera faces.
    void HandleMovement()
    {
        /*----------------------------------------------------------------
    * 1) Capture keyboard input in LOCAL space
    *    - Input.GetAxisRaw("Horizontal") returns −1, 0, or +1
    *      depending on A / D (or Left / Right arrow).
    *    - Input.GetAxisRaw("Vertical")   returns −1, 0, or +1
    *      for S / W (or Down / Up arrow).
    *    - Y component stays 0 because we don’t let the player fly.
    *    Example: pressing W+D  →  ( 1, 0, 1 )
    *---------------------------------------------------------------*/
        Vector3 input = new Vector3(
            Input.GetAxisRaw("Horizontal"),  // −1 … 0 … +1   (A / D)
            0f,
            Input.GetAxisRaw("Vertical"));   // W/S // −1 … 0 … +1   (S / W)
        /*----------------------------------------------------------------
  * 2) Normalise so diagonal isn’t faster
  *    Length of (1,0,1) is √2 ≈ 1.41. Without normalising the
  *    player would move 41 % faster on diagonals. Normalise rescales
  *    every non-zero vector to length 1 while keeping its direction.
  *---------------------------------------------------------------*/
        input.Normalize();
        /*----------------------------------------------------------------
   * 3) Convert from LOCAL (camera) space to WORLD space
   *    transform.TransformDirection rotates the input vector by the
   *    camera’s current yaw/pitch, so “forward” always means the
   *    direction we’re looking, not global +Z.
   *    Immediately multiply by moveSpeed to obtain a velocity vector
   *    of magnitude = moveSpeed (m/s).
   *    TransformDirection converts a direction from ‘forward according to me’ to ‘which way is that in the room’ so Unity can move the player correctly after they turn.
   *---------------------------------------------------------------*/
        Vector3 desired = transform.TransformDirection(input) * moveSpeed;
        /*----------------------------------------------------------------
    * 4) Smooth acceleration / deceleration
    *    velocity  ← lerp( current , desired ,  accel * Δt )
    *    - When the key is pressed, velocity eases toward ‘desired’.
    *    - When keys are released, velocity eases back to 0.
    *    The higher ‘acceleration’, the snappier the control.
    *---------------------------------------------------------------*/
        velocity = Vector3.Lerp(velocity, desired, acceleration * Time.deltaTime);
        /*----------------------------------------------------------------
    * 5) Move the CharacterController
    *    CharacterController expects a displacement, not velocity,
    *    so we multiply velocity by Δt again.
    *---------------------------------------------------------------*/
        controller.Move(velocity * Time.deltaTime);
    }
}

