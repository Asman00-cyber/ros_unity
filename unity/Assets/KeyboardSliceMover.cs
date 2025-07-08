using UnityEngine;

[RequireComponent(typeof(MeshRenderer))]
[RequireComponent(typeof(Images))]
public class KeyboardSliceMover : MonoBehaviour
{
    public int currentSliceIndex = 0;
    public float sliceSpacing = 0.01f;
    public float holdDelay = 0.3f;
    public float repeatRate = 0.05f;

    private Images images;
    private MeshRenderer meshRenderer;

    private float holdTimer = 0f;
    private float repeatTimer = 0f;
    private int direction = 0;

    void Start()
    {
        images = GetComponent<Images>();
        meshRenderer = GetComponent<MeshRenderer>();
        UpdateSlice();
    }

    void Update()
    {
        direction = 0;
        if (Input.GetKey(KeyCode.RightArrow)) direction = 1;
        if (Input.GetKey(KeyCode.LeftArrow)) direction = -1;

        if (direction != 0)
        {
            if (Input.GetKeyDown(KeyCode.RightArrow) || Input.GetKeyDown(KeyCode.LeftArrow))
            {
                StepSlice(direction);
                holdTimer = 0f;
                repeatTimer = 0f;
            }
            else
            {
                holdTimer += Time.deltaTime;
                if (holdTimer > holdDelay)
                {
                    repeatTimer += Time.deltaTime;
                    if (repeatTimer >= repeatRate)
                    {
                        StepSlice(direction);
                        repeatTimer = 0f;
                    }
                }
            }
        }
        else
        {
            holdTimer = 0f;
            repeatTimer = 0f;
        }
    }

    void StepSlice(int step)
    {
        currentSliceIndex += step;
        ClampIndex();
        UpdateSlice();
    }

    void ClampIndex()
    {
        if (Loader.Instance == null)
        {
            Debug.LogError("Loader.Instance is null in ClampIndex!");
            return;
        }

        switch (images.planeType)
        {
            case Images.ViewPlane.Axial:
                currentSliceIndex = Mathf.Clamp(currentSliceIndex, 0, Loader.Instance.depth - 1);
                break;
            case Images.ViewPlane.Sagittal:
                currentSliceIndex = Mathf.Clamp(currentSliceIndex, 0, Loader.Instance.width - 1);
                break;
            case Images.ViewPlane.Coronal:
                currentSliceIndex = Mathf.Clamp(currentSliceIndex, 0, Loader.Instance.height - 1);
                break;
        }
    }

    public void UpdateSlice()
    {
        if (images == null || meshRenderer == null)
        {
            Debug.LogWarning("Missing references in KeyboardSliceMover.");
            return;
        }

        if (Loader.Instance == null)
        {
            Debug.LogWarning("Loader.Instance is null in UpdateSlice!");
            return;
        }

        Debug.Log($"Plane: {images.planeType}, currentSliceIndex: {currentSliceIndex}");

        Texture2D sliceTexture = null;

        switch (images.planeType)
        {
            case Images.ViewPlane.Axial:
                if (Loader.Instance.axialSlices != null && currentSliceIndex >= 0 && currentSliceIndex < Loader.Instance.axialSlices.Length)
                {
                    Debug.Log($"Axial slices count: {Loader.Instance.axialSlices.Length}");
                    sliceTexture = Loader.Instance.axialSlices[currentSliceIndex];
                }
                break;
            case Images.ViewPlane.Sagittal:
                if (Loader.Instance.sagittalSlices != null && currentSliceIndex >= 0 && currentSliceIndex < Loader.Instance.sagittalSlices.Length)
                {
                    Debug.Log($"Sagittal slices count: {Loader.Instance.sagittalSlices.Length}");
                    sliceTexture = Loader.Instance.sagittalSlices[currentSliceIndex];
                }
                break;
            case Images.ViewPlane.Coronal:
                if (Loader.Instance.coronalSlices != null && currentSliceIndex >= 0 && currentSliceIndex < Loader.Instance.coronalSlices.Length)
                {
                    Debug.Log($"Coronal slices count: {Loader.Instance.coronalSlices.Length}");
                    sliceTexture = Loader.Instance.coronalSlices[currentSliceIndex];
                }
                break;
        }

        if (sliceTexture == null)
        {
            Debug.LogError($"Slice texture is null or index out of range! Plane: {images.planeType}, Index: {currentSliceIndex}");
            return;
        }

        meshRenderer.material.mainTexture = sliceTexture;
    }
}










