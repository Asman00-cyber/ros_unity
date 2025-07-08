using UnityEngine;
using System.IO;
using System.Linq;
using Dicom;
using Dicom.Imaging;
using System;

public class Loader : MonoBehaviour
{
    public static Loader Instance { get; private set; }

    public string dicomFolder = "Assets/ScalarVolume_18";

    public Texture2D[] axialSlices;
    public Texture2D[] sagittalSlices;
    public Texture2D[] coronalSlices;
    public int width, height, depth;

    ushort[,,] volumeData;

    void Awake()
    {
        if (Instance == null)
            Instance = this;
        else
            Destroy(gameObject);
    }

    void Start()
    {
        Debug.Log("Loader Start: Loading volume...");
        LoadVolume();
        GenerateSlices();
        Debug.Log($"Loader Start: Loaded volume size: {width}x{height}x{depth}");
    }

    void LoadVolume()
    {
        var files = Directory.GetFiles(dicomFolder, "*.dcm")
            .Select(f => new
            {
                Path = f,
                Name = Path.GetFileNameWithoutExtension(f),
                Index = ParseFlexibleIndex(Path.GetFileNameWithoutExtension(f))
            })
            .Where(x => x.Index.HasValue)
            .OrderBy(x => x.Index.Value)
            .Select(x => x.Path)
            .ToArray();

        if (files.Length == 0)
        {
            Debug.LogError("No valid DICOM files found in folder: " + dicomFolder);
            return;
        }

        var firstImg = new DicomImage(files[0]);
        width = firstImg.Width;
        height = firstImg.Height;
        depth = files.Length;

        volumeData = new ushort[width, height, depth];

        for (int z = 0; z < depth; z++)
        {
            var dicomImage = new DicomImage(files[z]);
            Texture2D tex = dicomImage.RenderImage().AsTexture2D();
            Color[] pixels = tex.GetPixels();

            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    float gray = pixels[y * width + x].grayscale;
                    ushort value = (ushort)(gray * 65535);
                    volumeData[x, y, z] = value;
                }
            }
        }

        Debug.Log("Volume loaded successfully.");
    }

    void GenerateSlices()
    {
        axialSlices = new Texture2D[depth];
        for (int z = 0; z < depth; z++)
        {
            axialSlices[z] = CreateTexture2D(width, height, (x, y) => volumeData[x, y, z]);
        }

        sagittalSlices = new Texture2D[width];
        for (int x = 0; x < width; x++)
        {
            sagittalSlices[x] = CreateTexture2D(depth, height, (z, y) => volumeData[x, y, z]);
        }

        coronalSlices = new Texture2D[height];
        for (int y = 0; y < height; y++)
        {
            coronalSlices[y] = CreateTexture2D(width, depth, (x, z) => volumeData[x, y, z]);
        }

        Debug.Log("Slices generated successfully.");
    }

    Texture2D CreateTexture2D(int w, int h, Func<int, int, ushort> getValue)
    {
        Texture2D tex = new Texture2D(w, h, TextureFormat.RGBA32, false);
        Color[] colors = new Color[w * h];

        for (int y = 0; y < h; y++)
        {
            for (int x = 0; x < w; x++)
            {
                float intensity = getValue(x, y) / 65535f;
                colors[y * w + x] = new Color(intensity, intensity, intensity, 1f);
            }
        }

        tex.SetPixels(colors);
        tex.Apply();
        return tex;
    }

    /// <summary>
    /// </summary>
    private long? ParseFlexibleIndex(string name)
    {
        // Try full parse 
        if (long.TryParse(name, out long full))
            return full;

        // Extract numeric part 
        string digits = new string(name.Where(char.IsDigit).ToArray());
        if (long.TryParse(digits, out long extracted))
            return extracted;

        // No valid number found
        return null;
    }
}
