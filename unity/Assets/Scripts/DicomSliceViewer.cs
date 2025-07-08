// Assets/Scripts/DicomSliceViewer.cs
using UnityEngine;
using Dicom;               // fo-dicom core
using Dicom.Imaging;
using System.IO;
using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting;

public class DicomSliceViewer : MonoBehaviour
{
    [Tooltip("Folder that contains the .dcm files (absolute or relative).")]
    public string dicomFolder = "Assets/CovidScans/Subject/Dicom";

    // Drag the Quad’s MeshRenderer *or* a UI RawImage here
    public Renderer targetRenderer;
    public UnityEngine.UI.RawImage targetRawImage;

    List<Texture2D> textures = new();
    int current = 0;

    void Start()
    {
        // 1. Collect *all* files that end with .dcm in the folder the user set.

        string[] files = Directory.GetFiles(dicomFolder, "*.dcm")
                          .OrderBy(f => int.Parse(Path.GetFileNameWithoutExtension(f))) //Path.GetFileNameWithoutExtension(f) → takes C:\Scans\123.dcm and keeps only 123.
                                                                                        //int.Parse(...) → turns the text "123" into the number 123.
                                                                                        //OrderBy( … ) → arranges all the paths from the smallest number to the biggest(1, 2, 3 …).
                          .ToArray();
        // 2. Convert each slice to a Texture2D
        foreach (var filepath in files)
        {
            print(filepath);
            DicomImage img = new DicomImage(filepath);
            textures.Add(img.RenderImage().AsTexture2D());
        }

        // 3. Show first slice
        ShowSlice(0);
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.RightArrow)) ShowSlice(current + 1);
        if (Input.GetKeyDown(KeyCode.LeftArrow)) ShowSlice(current - 1);
    }

    void ShowSlice(int index)
    {
        if (textures.Count == 0) return;
        current = Mathf.Clamp(index, 0, textures.Count - 1);

        if (targetRenderer != null)
            targetRenderer.material.mainTexture = textures[current];
        else if (targetRawImage != null)
            targetRawImage.texture = textures[current];
    }
}
