using UnityEngine;
using UnityEngine.UI;
using System.Collections;

/// <summary>
/// This is the code to get movement input from keyboard or controller into the PC.
/// It moves the player object on-screen using built-in physics, which is to say, it
/// applies a force to the PC, which responds accordingly.
/// </summary>
public class PlayerController : MonoBehaviour {

    public float speed;     
    private Rigidbody rb;

    /// <summary>
    /// Start() is called only once for any GameObject. Here, we want to retrieve
    /// the RigidBody and save it in variable rb. We do this now and save it so we
    /// don't have to retrieve it every frame, not a good practice.
    /// </summary>
    void Start() {
        rb = GetComponent<Rigidbody>();
    }

    /// <summary>
    /// This is called at the desired framerate, no matter what. This prevents having to
    /// take delta-T into accound as you would when using regular Update().
    /// Note the code for computing movement and applying forces; you may find that 
    /// useful later on.
    /// </summary>
    void FixedUpdate() {
        float moveHorizontal = Input.GetAxis("Horizontal");
        float moveVertical = Input.GetAxis("Vertical");

        // This simply moves the avatar based on arrow keys.
        // Note that the nose isn't getting correctly aligned. Use your SteeringBehavior to fix that.
        // Change speed on Inspector for "Red"
        // You could instead map this to the mouse if you like.
        this.transform.position = new Vector3(transform.position.x + speed * moveHorizontal, 1, transform.position.z + speed * moveVertical);

        // This is the physics based movement used in earlier assignments, not needed here.
        // Vector3 movement = new Vector3(moveHorizontal, 0.0f, moveVertical);
        // rb.AddForce(movement * speed);
    }

}