using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using System.Collections.Generic;
using Windows.Kinect;
using Joint = Windows.Kinect.Joint;

public class BodySourceView : MonoBehaviour 
{
    public Material BoneMaterial;
    public GameObject BodySourceManager;
    public GameObject mJointObject;
    public BodySourceManager _BodyManager;
    private Vector3[] positions = new Vector3[10];
    public float[] angles;
    public Text rs_flex;
    public Text rs_intex;
    public Text rs_abad;
    public Text rf_flex;
    public Text ls_flex;
    public Text ls_intex;
    public Text ls_abad;
    public Text lf_flex;
    public GameObject left_arm;
    public GameObject right_arm;
    public GameObject left_forearm;
    public GameObject right_forearm;
    public Dropdown Mode;
    public GameObject figure;
    public GameObject ghost_left_arm;
    public GameObject ghost_right_arm;
    public GameObject ghost_left_forearm;
    public GameObject ghost_right_forearm;
    public GameObject ghost_figure;
    public bool up = true;
    public float glef = 0f;
    public float gref = 0f;
    public float glier = 0f;
    public float grier = 0f;    
    public float glaa = 0f;
    public float graa = 0f;
    public float glfef = 0f;
    public float grfef = 0f;
    public Material skin;

    //abs add

    private Dictionary<ulong, GameObject> _Bodies = new Dictionary<ulong, GameObject>();
    
    
    /*private Dictionary<Kinect.JointType, Kinect.JointType> _BoneMap = new Dictionary<Kinect.JointType, Kinect.JointType>()
    {
        { Kinect.JointType.FootLeft, Kinect.JointType.AnkleLeft },
        { Kinect.JointType.AnkleLeft, Kinect.JointType.KneeLeft },
        { Kinect.JointType.KneeLeft, Kinect.JointType.HipLeft },
        { Kinect.JointType.HipLeft, Kinect.JointType.SpineBase },
        
        { Kinect.JointType.FootRight, Kinect.JointType.AnkleRight },
        { Kinect.JointType.AnkleRight, Kinect.JointType.KneeRight },
        { Kinect.JointType.KneeRight, Kinect.JointType.HipRight },
        { Kinect.JointType.HipRight, Kinect.JointType.SpineBase },
        
        { Kinect.JointType.HandTipLeft, Kinect.JointType.HandLeft },
        { Kinect.JointType.ThumbLeft, Kinect.JointType.HandLeft },
        { Kinect.JointType.HandLeft, Kinect.JointType.WristLeft },
        { Kinect.JointType.WristLeft, Kinect.JointType.ElbowLeft },
        { Kinect.JointType.ElbowLeft, Kinect.JointType.ShoulderLeft },
        { Kinect.JointType.ShoulderLeft, Kinect.JointType.SpineShoulder },
        
        { Kinect.JointType.HandTipRight, Kinect.JointType.HandRight },
        { Kinect.JointType.ThumbRight, Kinect.JointType.HandRight },
        { Kinect.JointType.HandRight, Kinect.JointType.WristRight },
        { Kinect.JointType.WristRight, Kinect.JointType.ElbowRight },
        { Kinect.JointType.ElbowRight, Kinect.JointType.ShoulderRight },
        { Kinect.JointType.ShoulderRight, Kinect.JointType.SpineShoulder },
        
        { Kinect.JointType.SpineBase, Kinect.JointType.SpineMid },
        { Kinect.JointType.SpineMid, Kinect.JointType.SpineShoulder },
        { Kinect.JointType.SpineShoulder, Kinect.JointType.Neck },
        { Kinect.JointType.Neck, Kinect.JointType.Head },
    };*/


    private List<JointType> joints_ = new List<JointType> {
        JointType.WristLeft,
        JointType.ElbowLeft,
        JointType.ShoulderLeft,
        JointType.WristRight,
        JointType.ElbowRight,
        JointType.ShoulderRight,
        JointType.SpineBase,
        JointType.SpineShoulder,
        JointType.HipRight,
        JointType.HipLeft,
    };
    void Update () 
    {
        if (BodySourceManager == null)
        {
            return;
        }
        
        _BodyManager = BodySourceManager.GetComponent<BodySourceManager>();
        if (_BodyManager == null)
        {
            return;
        }
        
        Body[] data = _BodyManager.GetData();
        if (data == null)
        {
            return;
        }
        
        List<ulong> trackedIds = new List<ulong>();
        foreach(var body in data)
        {
            if (body == null)
            {
                continue;
              }
                
            if(body.IsTracked)
            {
                trackedIds.Add (body.TrackingId);
            }
        }
        
        List<ulong> knownIds = new List<ulong>(_Bodies.Keys);
        
        // First delete untracked bodies
        foreach(ulong trackingId in knownIds)
        {
            if(!trackedIds.Contains(trackingId))
            {
                Destroy(_Bodies[trackingId]);
                _Bodies.Remove(trackingId);
            }
        }

        foreach(var body in data)
        {
            if (body == null)
            {
                continue;
            }
            
            if(body.IsTracked)
            {
                if(!_Bodies.ContainsKey(body.TrackingId))
                {
                    _Bodies[body.TrackingId] = CreateBodyObject(body.TrackingId);
                }
                
                RefreshBodyObject(body, _Bodies[body.TrackingId]);
            }
        }
    }
    
    private GameObject CreateBodyObject(ulong id)
    {
        GameObject body = new GameObject("Body:" + id);
        
        foreach(JointType joint in joints_)
        {
            GameObject newJoint = Instantiate(mJointObject);
            newJoint.name = joint.ToString();

            newJoint.transform.parent = body.transform;
        }
        /*for (Kinect.JointType jt = Kinect.JointType.SpineBase; jt <= Kinect.JointType.ThumbRight; jt++)
        {
            GameObject jointObj = GameObject.CreatePrimitive(PrimitiveType.Cube);
            
            LineRenderer lr = jointObj.AddComponent<LineRenderer>();
            lr.SetVertexCount(2);
            lr.material = BoneMaterial;
            lr.SetWidth(0.05f, 0.05f);
            
            jointObj.transform.localScale = new Vector3(0.3f, 0.3f, 0.3f);
            jointObj.name = jt.ToString();
            jointObj.transform.parent = body.transform;
        }*/
        
        return body;
    }
    
    private void RefreshBodyObject(Body body, GameObject bodyObject)
    {
        int count = 0;
        foreach (JointType joint in joints_)
        {
            Joint sourceJoint = body.Joints[joint];
            Vector3 targetPosition = GetVector3FromJoint(sourceJoint);           

            Transform jointObject = bodyObject.transform.Find(joint.ToString());
            jointObject.position = targetPosition;
            positions[count] = targetPosition;
            count += 1;
        }
        angles = Get_Kinect(positions);
        int mode = Mode.value;
        switch (mode)
        {
            //ghost_
            case 0: // Left flex extension
                left_arm.transform.localRotation = Quaternion.Euler(-angles[0], 0f, 90.0f);
                left_forearm.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);
                right_arm.transform.localRotation = Quaternion.Euler(0f, 0f, -90.0f);
                right_forearm.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);
                figure.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);
                ghost_figure.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);
                if (up)
                {
                    glef -= 0.5f;
                    if (glef <= -200.0f)
                    {
                        up = false;
                    }
                }                
                if(!up)
                {
                    glef += 0.5f;
                    if(glef >= 20.0f)
                    {
                        up = true;
                    }
                }            
                ghost_left_arm.transform.localRotation = Quaternion.Euler(0f + glef, 0f, 90.0f);
                ghost_left_forearm.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);
                ghost_right_arm.transform.localRotation = Quaternion.Euler(0f, 0f, -90.0f);
                ghost_right_forearm.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);

                if (Mathf.Abs(angles[0] - glef) <= 10)
                {

                    skin.SetColor("_Color", Color.green);
                }
                else
                {
                    skin.SetColor("_Color", Color.red);
                }


                break;
            case 1: //Left internal external rotation
                left_arm.transform.localRotation = Quaternion.Euler(0f, 0f, 90.0f);
                left_forearm.transform.localRotation = Quaternion.Euler(0f, 90f, angles[4]);
                right_arm.transform.localRotation = Quaternion.Euler(0f, 0f, -90.0f);
                right_forearm.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);
                figure.transform.localRotation = Quaternion.Euler(0f, -90f, 0f);
                ghost_figure.transform.localRotation = Quaternion.Euler(0f, -90f, 0f);
                if (up)
                {
                    glier -= 0.5f;
                    if (glier <= -75.0f)
                    {
                        up = false;
                    }
                }
                if (!up)
                {
                    glier += 0.5f;
                    if (glier >= 50.0f)
                    {
                        up = true;
                    }
                }
                ghost_left_arm.transform.localRotation = Quaternion.Euler(0f, 0f, 90.0f);
                ghost_left_forearm.transform.localRotation = Quaternion.Euler(0f, 90f, glier);
                ghost_right_arm.transform.localRotation = Quaternion.Euler(0f, 0f, -90.0f);
                ghost_right_forearm.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);

                if (Mathf.Abs(angles[4] - glier) <= 10)
                {

                    skin.SetColor("_Color", Color.green);
                }
                else
                {
                    skin.SetColor("_Color", Color.red);
                }



                break;
            case 2: //Left abduction adduction
                left_arm.transform.localRotation = Quaternion.Euler(-angles[2], -90f, 90f);
                left_forearm.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);
                right_arm.transform.localRotation = Quaternion.Euler(0f, 0f, -90.0f);
                right_forearm.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);
                figure.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);
                ghost_figure.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);
                if (up)
                {
                    glaa += 0.5f;
                    if (glaa >= 180.0f)
                    {
                        up = false;
                    }
                }
                if (!up)
                {
                    glaa -= 0.5f;
                    if (glaa <= 5.0f)
                    {
                        up = true;
                    }
                }
                ghost_left_arm.transform.localRotation = Quaternion.Euler(-glaa, -90f, 90f);
                ghost_left_forearm.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);
                ghost_right_arm.transform.localRotation = Quaternion.Euler(0f, 0f, -90.0f);
                ghost_right_forearm.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);

                if (Mathf.Abs(angles[2] - glaa) <= 10)
                {

                    Debug.Log("it worked!");
                    skin.SetColor("_Color", Color.green);
                }
                else
                {
                    skin.SetColor("_Color", Color.red);
                }

                break;
            case 3: //Left forearm fe
                left_arm.transform.localRotation = Quaternion.Euler(0f, 0f, 90.0f);
                left_forearm.transform.localRotation = Quaternion.Euler(0f, angles[6], 0f);
                right_arm.transform.localRotation = Quaternion.Euler(0f, 0f, -90.0f);
                right_forearm.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);
                figure.transform.localRotation = Quaternion.Euler(0f, -45f, 0f);
                ghost_figure.transform.localRotation = Quaternion.Euler(0f, -45f, 0f);
                if (up)
                {
                    glfef += 0.5f;
                    if (glfef >= 145.0f)
                    {
                        up = false;
                    }
                }
                if (!up)
                {
                    glfef -= 0.5f;
                    if (glfef <= 5.0f)
                    {
                        up = true;
                    }
                }
                ghost_left_arm.transform.localRotation = Quaternion.Euler(0f, 0f, 90.0f);
                ghost_left_forearm.transform.localRotation = Quaternion.Euler(0f, glfef, 0f);
                ghost_right_arm.transform.localRotation = Quaternion.Euler(0f, 0f, -90.0f);
                ghost_right_forearm.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);

                if (Mathf.Abs(angles[6] - glfef) <= 10)
                {

                    skin.SetColor("_Color", Color.green);
                }
                else
                {
                    skin.SetColor("_Color", Color.red);
                }


                break;
            case 4: // right flex extension
                right_arm.transform.localRotation = Quaternion.Euler(-angles[1], 0.0f, -90.0f);
                right_forearm.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);
                left_arm.transform.localRotation = Quaternion.Euler(0f, 0f, 90.0f);
                left_forearm.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);
                figure.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);
                ghost_figure.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);
                if (up)
                {
                    gref -= 0.5f;
                    if (gref <= -200.0f)
                    {
                        up = false;
                    }
                }
                if (!up)
                {
                    gref += 0.5f;
                    if (gref >= 20.0f)
                    {
                        up = true;
                    }
                }
                ghost_right_arm.transform.localRotation = Quaternion.Euler(gref, 0f, -90.0f);
                ghost_right_forearm.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);
                ghost_left_arm.transform.localRotation = Quaternion.Euler(0f, 0f, 90.0f);
                ghost_left_forearm.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);

                if (Mathf.Abs(angles[1] - (-gref)) <= 10)
                {

                    skin.SetColor("_Color", Color.green);
                }
                else
                {
                    skin.SetColor("_Color", Color.red);
                }

                break;
            case 5: //right internal external rotation
                right_arm.transform.localRotation = Quaternion.Euler(0f, 0f, -90.0f);
                right_forearm.transform.localRotation = Quaternion.Euler(0f, -90f, angles[5]);
                left_arm.transform.localRotation = Quaternion.Euler(0f, 0f, 90.0f);
                left_forearm.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);
                figure.transform.localRotation = Quaternion.Euler(0f, 90f, 0f);
                ghost_figure.transform.localRotation = Quaternion.Euler(0f, 90f, 0f);
                if (up)
                {
                    grier -= 0.5f;
                    if (grier <= -75.0f)
                    {
                        up = false;
                    }
                }
                if (!up)
                {
                    grier += 0.5f;
                    if (grier >= 40.0f)
                    {
                        up = true;
                    }
                }
                ghost_right_arm.transform.localRotation = Quaternion.Euler(0f, 0f, -90.0f);
                ghost_right_forearm.transform.localRotation = Quaternion.Euler(0f, -90f, -grier);
                ghost_left_arm.transform.localRotation = Quaternion.Euler(0f, 0f, 90.0f);
                ghost_left_forearm.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);
                if (Mathf.Abs(angles[5] - (-grier)) <= 10)
                {

                    skin.SetColor("_Color", Color.green);
                }
                else
                {
                    skin.SetColor("_Color", Color.red);
                }

                break;
            case 6: //right abduction adduction
                right_forearm.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);
                right_arm.transform.localRotation = Quaternion.Euler(angles[3], 90f, -90f);
                left_arm.transform.localRotation = Quaternion.Euler(0f, 0f, 90.0f);
                left_forearm.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);
                figure.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);
                ghost_figure.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);
                if (up)
                {
                    graa += 0.5f;
                    if (graa >= 180.0f)
                    {
                        up = false;
                    }
                }
                if (!up)
                {
                    graa -= 0.5f;
                    if (graa <= 5.0f)
                    {
                        up = true;
                    }
                }
                ghost_right_forearm.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);
                ghost_right_arm.transform.localRotation = Quaternion.Euler(-graa, 90f, -90f);
                ghost_left_arm.transform.localRotation = Quaternion.Euler(0f, 0f, 90.0f);
                ghost_left_forearm.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);
                if (Mathf.Abs(angles[3] - (-graa)) <= 10)
                {

                    skin.SetColor("_Color", Color.green);
                    Debug.Log("green");
                }
                else
                {
                    skin.SetColor("_Color", Color.red);
                }

                break;
            case 7: //right forearm fe
                right_arm.transform.localRotation = Quaternion.Euler(0f, 0f, -90.0f);
                right_forearm.transform.localRotation = Quaternion.Euler(0f, -angles[7], 0f);
                left_arm.transform.localRotation = Quaternion.Euler(0f, 0f, 90.0f);
                left_forearm.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);
                figure.transform.localRotation = Quaternion.Euler(0f, 45f, 0f);
                ghost_figure.transform.localRotation = Quaternion.Euler(0f, 45f, 0f);
                if (up)
                {
                    grfef += 0.5f;
                    if (grfef >= 145.0f)
                    {
                        up = false;
                    }
                }
                if (!up)
                {
                    grfef -= 0.5f;
                    if (grfef <= 5.0f)
                    {
                        up = true;
                    }
                }
                ghost_right_arm.transform.localRotation = Quaternion.Euler(0f, 0f, -90.0f);
                ghost_right_forearm.transform.localRotation = Quaternion.Euler(0f, -grfef, 0f);
                ghost_left_arm.transform.localRotation = Quaternion.Euler(0f, 0f, 90.0f);
                ghost_left_forearm.transform.localRotation = Quaternion.Euler(0f, 0f, 0f);

                if (Mathf.Abs(angles[7] - grfef) <= 10)
                {

                    skin.SetColor("_Color", Color.green);
                }
                else
                {
                    skin.SetColor("_Color", Color.red);
                }

                break;
        }
        
    }

    private static Color GetColorForState(TrackingState state)
    {
        switch (state)
        {
        case TrackingState.Tracked:
            return Color.green;

        case TrackingState.Inferred:
            return Color.red;

        default:
            return Color.black;
        }
    }
    
    private static Vector3 GetVector3FromJoint(Joint joint)
    {
        return new Vector3(joint.Position.X * 10, joint.Position.Y * 10, joint.Position.Z * 10);
    }
    static int GetMaxIndex(float[] theArray)
    {
        int maxIndex = 0;
        for (int i = 0; i < theArray.Length; i++)
        {
            if (theArray[i] > theArray[maxIndex]) maxIndex = i;
        }
        return maxIndex;
    }
    public float[] Get_Kinect(Vector3[] positions)
    {
        //Left Joints
        Vector3 leftShoulder = positions[2];
        Vector3 leftElbow = positions[1];
        Vector3 leftWrist = positions[0];
        //Right Joints
        Vector3 rightShoulder = positions[5];
        Vector3 rightElbow = positions[4];
        Vector3 rightWrist = positions[3];
        Vector3 rightHip = positions[8];
        Vector3 leftHip = positions[9];
        //Spine Joints
        Vector3 spineBase = positions[6];
        Vector3 spineShoulder = positions[7];
        //Back Reference
        Vector3 TrunkVector = spineBase - spineShoulder;
        Vector3 RSLS = (rightHip - leftHip);
        RSLS = RSLS.normalized;
        //Normal to transversal plane
        Vector3 trans_X = (TrunkVector / (TrunkVector).magnitude);
        //normal to saggital plane
        Vector3 sag_Y = ((RSLS - Vector3.Dot(RSLS, trans_X) * trans_X) / (RSLS - Vector3.Dot(RSLS, trans_X) * trans_X).magnitude);
        //normal to coronal plane 
        Vector3 cor_Z = (Vector3.Cross(trans_X, sag_Y));
        cor_Z = cor_Z.normalized;
        //Debug.Log("Transx: " + trans_X);
        //Debug.Log("sagy: " + sag_Y);
        //Debug.Log("corz: " + cor_Z);
        //Shoulder orientation computation
        Vector3 L_arm = leftElbow - leftShoulder;
        L_arm = L_arm.normalized;        
        L_arm = new Vector3(Vector3.Dot(L_arm, trans_X), Vector3.Dot(L_arm, -sag_Y), Vector3.Dot(L_arm, -cor_Z));
        //Debug.Log(L_arm);
        Vector3 R_arm = rightElbow - rightShoulder;
        R_arm = R_arm.normalized;
        R_arm = new Vector3(Vector3.Dot(R_arm, trans_X), Vector3.Dot(R_arm, -sag_Y), Vector3.Dot(R_arm, -cor_Z));
        //Shoulder extension flexion
        float lef = Mathf.Atan2(L_arm[2], L_arm[0]);
        lef = lef * Mathf.Rad2Deg;
        if (lef >= -180 && lef <= -150) {
            lef = 360 + lef;
        }
        float ref_ = Mathf.Atan2(R_arm[2], R_arm[0]);
        ref_ = ref_ * Mathf.Rad2Deg;
        if (ref_ >= -180 && ref_ <= -150)
        {
            ref_ = 360 + ref_;
        }
        //Shoulder abduction adduction
        float lbd = Mathf.Atan2(L_arm[1], L_arm[0]);
        lbd = lbd * Mathf.Rad2Deg;
        if (lbd >= -180 && lbd <= -150)
        {
            lbd = 360 + lbd;
        }                    
        
        float rbd = Mathf.Atan2(R_arm[1], R_arm[0]);
        rbd = rbd * Mathf.Rad2Deg;
        if (rbd >= -180 && rbd <= -150)
        {
            rbd = 360 + rbd;
        }
        //Elbow joint angle calculation
        Vector3 LA = (leftElbow - leftShoulder).normalized;
        Vector3 LFA = (leftWrist - leftElbow).normalized;
        float lelb = Mathf.Acos(Vector3.Dot(LA, LFA));
        lelb = lelb * Mathf.Rad2Deg;
        Vector3 RA = (rightElbow - rightShoulder).normalized;
        Vector3 RFA = (rightWrist - rightElbow).normalized;
        float relb = Mathf.Acos(Vector3.Dot(RA, RFA));
        relb = relb * Mathf.Rad2Deg;

        float lie = 666;
        if (lelb > 30)
        {
            Vector3 PP = new Vector3(Vector3.Dot(LA, trans_X), Vector3.Dot(LA, sag_Y), Vector3.Dot(LA, cor_Z));
            Vector3 AbsPP = new Vector3(Mathf.Abs(PP.x), Mathf.Abs(PP.y), Mathf.Abs(PP.z));
            Vector3 Zref;
            Vector3 Yref;
            Vector3 Xref;

            float x_cord = AbsPP.x;
            float y_cord = AbsPP.y;
            float z_cord = AbsPP.z;

            float[] vectorcord = { x_cord, y_cord, z_cord };
            int ind = GetMaxIndex(vectorcord);

            switch (ind)
            {
                case 0:
                    Zref = -(cor_Z - Vector3.Dot(cor_Z, LA) * LA);
                    Zref = Zref.normalized;
                    Yref = sag_Y - Vector3.Dot(sag_Y, LA) * LA;
                    Yref = Yref.normalized;
                    lie = Mathf.Atan2(Vector3.Dot(LFA, Yref), Vector3.Dot(LFA, Zref));
                    lie = lie * Mathf.Rad2Deg;
                    break;
                case 1:
                    Zref = -(cor_Z - Vector3.Dot(cor_Z, LA) * LA);
                    Zref = Zref.normalized;
                    Xref = trans_X - Vector3.Dot(trans_X, LA) * LA;
                    Xref = Xref.normalized;
                    lie = Mathf.Atan2(Vector3.Dot(LFA, Xref), Vector3.Dot(LFA, Zref));
                    lie = lie * Mathf.Rad2Deg;
                    break;
                case 2:
                    Xref = trans_X - Vector3.Dot(trans_X, LA) * LA;
                    Xref = -Xref.normalized;
                    Yref = sag_Y - Vector3.Dot(sag_Y, LA) * LA;
                    Yref = Yref.normalized;
                    lie = Mathf.Atan2(Vector3.Dot(LFA, Yref), Vector3.Dot(LFA, Xref));
                    lie = lie * Mathf.Rad2Deg;
                    break;
            } 
        }
        float rie = 666;
        if (relb > 30)
        {
            Vector3 rPP = new Vector3(Vector3.Dot(RA, trans_X), Vector3.Dot(RA, sag_Y), Vector3.Dot(RA, cor_Z));
            Vector3 rAbsPP = new Vector3(Mathf.Abs(rPP.x), Mathf.Abs(rPP.y), Mathf.Abs(rPP.z));
            Vector3 rZref;
            Vector3 rYref;
            Vector3 rXref;

            float rx_cord = rAbsPP.x;
            float ry_cord = rAbsPP.y;
            float rz_cord = rAbsPP.z;

            float[] rvectorcord = { rx_cord, ry_cord, rz_cord };
            int idx = GetMaxIndex(rvectorcord);
            

            switch (idx)
            {

                case 0:
                    rZref = -(cor_Z - Vector3.Dot(cor_Z, RA) * RA);
                    rZref = rZref.normalized;
                    rYref = sag_Y - Vector3.Dot(sag_Y, RA) * RA;
                    rYref = rYref.normalized;
                    rie = Mathf.Atan2(Vector3.Dot(RFA, rYref), Vector3.Dot(RFA, rZref));
                    rie = rie * Mathf.Rad2Deg;
                    break;
                case 1:
                    rZref = -(cor_Z - Vector3.Dot(cor_Z, RA) * RA);
                    rZref = rZref.normalized;
                    rXref = trans_X - Vector3.Dot(trans_X, RA) * RA;
                    rXref = rXref.normalized;
                    rie = Mathf.Atan2(Vector3.Dot(RFA, rXref), Vector3.Dot(RFA, rZref));
                    rie = rie * Mathf.Rad2Deg;
                    break;
                case 2:
                    rXref = trans_X - Vector3.Dot(trans_X, RA) * RA;
                    rXref = -rXref.normalized;
                    rYref = sag_Y - Vector3.Dot(sag_Y, RA) * RA;
                    rYref = rYref.normalized;
                    rie = Mathf.Atan2(Vector3.Dot(RFA, rYref), Vector3.Dot(RFA, rXref));
                    rie = rie * Mathf.Rad2Deg;
                    break;

            }
        }
        float[] kinect_ang = { lef, ref_, lbd, rbd, lie, rie, lelb, relb };
        //Right Text
        rs_flex.text = "Flex-Ext: " + ref_.ToString();
        rs_intex.text = "Int-Ex Rotation: " + rie.ToString();
        rs_abad.text = "Abd-Add: " + rbd.ToString();
        rf_flex.text = "Flex-Ext: " + relb.ToString();
        //Left Text
        ls_flex.text = "Flex-Ext: " + lef.ToString();
        ls_intex.text = "Int-Ex Rotation: " + lie.ToString();
        ls_abad.text = "Abd-Add: " + lbd.ToString();
        lf_flex.text = "Flex-Ext: " + lelb.ToString();

        return kinect_ang;
    }        
} 