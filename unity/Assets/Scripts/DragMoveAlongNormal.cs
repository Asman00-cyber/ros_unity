// DragMoveAlongNormal.cs
//
// ▸ Attach to the Quad you want to drag.
// ▸ The object needs a Collider (Quad has one by default).
// ▸ While the left mouse button is held on the Quad, move the mouse up / down
//   to push or pull the Quad along its local normal.

using UnityEngine;

[RequireComponent(typeof(Collider))]
public class DragMoveAlongNormal : MonoBehaviour
{
    [Tooltip("Speed multiplier for mouse movement to distance.")]
    public float dragSensitivity = 0.01f;

    // Private bookkeeping
    bool isDragging;
    float startScreenY;     // mouse Y when drag began
    Vector3 startLocalPos;   // localPosition when drag began
    Vector3 localNormal;     // cached local-space normal (+Z for Quad)

    void Awake()
    {
        // For a Unity Quad the forward direction is +Z in local space
        localNormal = Vector3.forward;
    }

    // Called by Unity when the user presses the mouse button over this collider
    void OnMouseDown()
    {
        isDragging = true;
        startScreenY = Input.mousePosition.y;
        startLocalPos = transform.localPosition;
    }

    // Called every frame while the mouse is held down on this collider
    void OnMouseDrag()
    {
        if (!isDragging) return;

        float deltaY = Input.mousePosition.y - startScreenY;
        float move = deltaY * dragSensitivity;

        // Move along the local normal (positive = forward)
        transform.localPosition = startLocalPos + localNormal * move;
    }

    // Called when the mouse button is released
    void OnMouseUp()
    {
        isDragging = false;
    }
}
