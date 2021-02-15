using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

using System.Linq;
using System.Text;
using System.Threading.Tasks;
using OpenCvSharp;

public class IMG_Test : MonoBehaviour
{
    public RenderTexture IMG;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        GameObject image = GameObject.Find("IMG");
        RawImage Result = image.GetComponent<RawImage>();

        //Result.texture = toTexture(IMG);
        Result.texture = OpenCvSharp.Unity.MatToTexture(Test());
    }

    byte[] toPNG(RenderTexture rTex)
    {
        Texture2D tex = new Texture2D(256, 256, TextureFormat.RGB24, false);
        RenderTexture.active = rTex;
        tex.ReadPixels(new UnityEngine.Rect(0, 0, rTex.width, rTex.height), 0, 0);
        tex.Apply();
        byte[] Output = tex.EncodeToPNG();
        return Output;
    }

    Texture2D toTexture(RenderTexture rTex)
    {
        Texture2D tex = new Texture2D(256, 256, TextureFormat.RGB24, false);
        RenderTexture.active = rTex;
        tex.ReadPixels(new UnityEngine.Rect(0, 0, rTex.width, rTex.height), 0, 0);
        tex.Apply();
        
        return tex;
    }

    Mat Test()
    {
        Mat RGB_IMG = OpenCvSharp.Unity.TextureToMat(toTexture(IMG));
        Mat GRAY_IMG = new Mat ();
        Mat Canny_Edge = new Mat ();
        Cv2.CvtColor(RGB_IMG,GRAY_IMG,ColorConversionCodes.BGR2GRAY);
        Cv2.Canny (GRAY_IMG, Canny_Edge, 10.0, 70.0);

        LineSegmentPolar[] Line_Spec = Cv2.HoughLines(Canny_Edge,1,Math.PI/180,130,0,0);
        
        try
        {
            string S_Rho = Line_Spec[0].Rho.ToString();
            string S_Theta = Line_Spec[0].Theta.ToString();

            Debug.Log(S_Rho + "  ,  " + S_Theta);
        }
        catch(Exception e)
        {
            //Debug.Log(e);
        }
        return Canny_Edge;
    }
    


    


}
