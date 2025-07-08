using UnityEngine;
using Dicom;
using Dicom.Imaging;

/// <summary>Attach this to an empty GameObject.  
/// Drop a Quad (MeshRenderer) **or** RawImage into the appropriate slot.</summary>
public class DicomLoader : MonoBehaviour
{
    [Tooltip("Absolute or relative path (inside Assets) to the .dcm file")]
    public string dicomPath = "Assets/CovidScans/Subject/Dicom/sample.dcm";

    // One of these will be used:
    public Renderer targetRenderer;   // for a Quad / Plane
    public UnityEngine.UI.RawImage targetRawImage; // for a UI RawImage

    void Start()
    {
        // 1. Load the DICOM
        var image = new DicomImage(dicomPath);

        // 2. Render to Texture2D (Unity-specific helper from fo-dicom)
        Texture2D tex = image.RenderImage().AsTexture2D();

        // 3. Push the texture to whatever display surface we have
        if (targetRenderer != null)
            targetRenderer.material.mainTexture = tex;
        else if (targetRawImage != null)
            targetRawImage.texture = tex;
        else
            Debug.LogWarning("No display surface assigned for DICOM texture.");
    }
}
