using UnityEngine;
using System.Collections;

[RequireComponent(typeof(Images))]
[RequireComponent(typeof(KeyboardSliceMover))]
public class SliceClickSwitcher : MonoBehaviour
{
    public Camera mainCamera;
    private Images images;
    private KeyboardSliceMover keyboardMover;
    public CoordinatesPublisher voxelpublisher; // call the coordinate publisher class
    // Store clicked pixel coordinates and slice indices for each plane
    private Vector2Int axialClick = Vector2Int.zero;
    private Vector2Int sagittalClick = Vector2Int.zero;
    private Vector2Int coronalClick = Vector2Int.zero;

    private int axialSliceIndex = 0;
    private int sagittalSliceIndex = 0;
    private int coronalSliceIndex = 0;

    private int step = 0;

    void Start()
    {
        images = GetComponent<Images>();
        keyboardMover = GetComponent<KeyboardSliceMover>();
        GameObject publisherobject= GameObject.Find("Publish");
        if (publisherobject!=null)
        {
            voxelpublisher = publisherobject.GetComponent<CoordinatesPublisher>();
        }
        else
        {
            Debug.Log("No component inside the Publish object was found!");
        }

        if (mainCamera == null)
            mainCamera = Camera.main;
        
        

        StartCoroutine(WaitForLoaderAndSetInitialPlane());
    }

    IEnumerator WaitForLoaderAndSetInitialPlane()
    {
        while (Loader.Instance == null || Loader.Instance.axialSlices == null || Loader.Instance.axialSlices.Length == 0)
        {
            yield return null; // Wait one frame
        }

        SetPlane(Images.ViewPlane.Axial);
    }

    void Update()
    {
        if (Input.GetMouseButtonDown(0))
        {
            HandleClick();
        }
    }

    void HandleClick()
    {
        Ray ray = mainCamera.ScreenPointToRay(Input.mousePosition);
        if (Physics.Raycast(ray, out RaycastHit hit))
        {
            Vector2 uv = hit.textureCoord;

            switch (images.planeType)
            {
                case Images.ViewPlane.Axial:
                    axialClick = new Vector2Int(
                        Mathf.RoundToInt(uv.x * Loader.Instance.width),
                        Mathf.RoundToInt(uv.y * Loader.Instance.height)
                    );
                    axialSliceIndex = keyboardMover.currentSliceIndex;
                    Debug.Log($"Axial click: pixel (x,y) = ({axialClick.x},{axialClick.y}), slice = {axialSliceIndex}");
                    break;

                case Images.ViewPlane.Sagittal:
                    sagittalClick = new Vector2Int(
                        Mathf.RoundToInt(uv.x * Loader.Instance.depth),
                        Mathf.RoundToInt(uv.y * Loader.Instance.height)
                    );
                    sagittalSliceIndex = keyboardMover.currentSliceIndex;
                    Debug.Log($"Sagittal click: pixel (z,y) = ({sagittalClick.x},{sagittalClick.y}), slice = {sagittalSliceIndex}");
                    break;

                case Images.ViewPlane.Coronal:
                    coronalClick = new Vector2Int(
                        Mathf.RoundToInt(uv.x * Loader.Instance.width),
                        Mathf.RoundToInt(uv.y * Loader.Instance.depth)
                    );
                    coronalSliceIndex = keyboardMover.currentSliceIndex;
                    Debug.Log($"Coronal click: pixel (x,z) = ({coronalClick.x},{coronalClick.y}), slice = {coronalSliceIndex}");
                    break;
            }

            step++;

            if (step == 1)
                SetPlane(Images.ViewPlane.Sagittal);
            else if (step == 2)
                SetPlane(Images.ViewPlane.Coronal);
            else if (step == 3)
                FinalizeSelection();
        }
    }

    void SetPlane(Images.ViewPlane plane)
    {
        images.planeType = plane;
        keyboardMover.currentSliceIndex = 0;
        keyboardMover.UpdateSlice();
    }


    void FinalizeSelection()
    {
        // Axial view:
        int x_from_axial = axialClick.x;
        int y_from_axial = axialClick.y;
        int z_from_axial = axialSliceIndex;

        // Sagittal view:
        int x_from_sagittal = sagittalSliceIndex;
        int y_from_sagittal = sagittalClick.y;
        int z_from_sagittal = sagittalClick.x;

        // Coronal view:
        int x_from_coronal = coronalClick.x;
        int y_from_coronal = coronalSliceIndex;
        int z_from_coronal = coronalClick.y;

        // Average
        int finalX = Mathf.RoundToInt((x_from_axial + x_from_sagittal + x_from_coronal) / 3f);
        int finalY = Mathf.RoundToInt((y_from_axial + y_from_sagittal + y_from_coronal) / 3f);
        int finalZ = Mathf.RoundToInt((z_from_axial + z_from_sagittal + z_from_coronal) / 3f);

        Vector3Int finalCoordinate = new Vector3Int(finalX, finalY, finalZ);

        Debug.Log($" final voxel coordinate: {finalCoordinate}");
        if(voxelpublisher!=null)
        {
            voxelpublisher.PublishCoordinates(finalCoordinate);
        }
        else
        {
            Debug.LogWarning($"Coordinate publisher not set in SliceClickSwitcher\n");
        }
        step = 0;
        SetPlane(Images.ViewPlane.Axial);
    }

}






