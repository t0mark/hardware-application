using UnityEngine;

/// <summary>
/// VR 씬에서 카메라 스트림을 표시하는 화면 오브젝트를 관리합니다.
/// Head-locked 모드: 항상 시야 정면에 고정
/// World-locked 모드: 처음 위치에 고정 (기본값)
/// </summary>
[RequireComponent(typeof(MJPEGStreamReceiver))]
public class CameraViewScreen : MonoBehaviour
{
    [Header("Screen Transform")]
    [Tooltip("카메라 화면까지의 거리 (m)")]
    public float distance = 2.0f;

    [Tooltip("화면 크기 (가로 x 세로, m)")]
    public Vector2 screenSize = new Vector2(1.6f, 0.9f); // 16:9

    [Tooltip("화면 높이 오프셋 (m, 양수=위)")]
    public float heightOffset = 0.0f;

    [Header("Display Mode")]
    [Tooltip("true: 월드 고정 (고개 돌려도 제자리) / false: 항상 시야 정면에 따라옴")]
    public bool worldLocked = true;

    [Tooltip("Head-locked 모드에서 부드럽게 따라오는 속도")]
    public float followSpeed = 5.0f;

    [Header("References")]
    public Camera vrCamera;

    private GameObject _screenQuad;
    private MJPEGStreamReceiver _receiver;
    private static readonly int BaseMap = Shader.PropertyToID("_BaseMap");

    void Awake()
    {
        _receiver = GetComponent<MJPEGStreamReceiver>();
    }

    void Start()
    {
        if (vrCamera == null)
            vrCamera = Camera.main;

        CreateScreen();
    }

    void Update()
    {
        if (_screenQuad == null) return;

        if (!worldLocked)
        {
            // 시야 정면에 부드럽게 따라오기
            Vector3 targetPos = vrCamera.transform.position
                + vrCamera.transform.forward * distance
                + Vector3.up * heightOffset;
            Quaternion targetRot = Quaternion.LookRotation(
                _screenQuad.transform.position - vrCamera.transform.position);

            _screenQuad.transform.position = Vector3.Lerp(
                _screenQuad.transform.position, targetPos, Time.deltaTime * followSpeed);
            _screenQuad.transform.rotation = Quaternion.Slerp(
                _screenQuad.transform.rotation, targetRot, Time.deltaTime * followSpeed);
        }

    }

    private void CreateScreen()
    {
        _screenQuad = GameObject.CreatePrimitive(PrimitiveType.Quad);
        _screenQuad.name = "CameraViewScreen";

        // 크기 설정
        _screenQuad.transform.localScale = new Vector3(screenSize.x, screenSize.y, 1f);

        // 초기 위치 설정 (카메라 정면)
        if (vrCamera != null)
        {
            _screenQuad.transform.position = vrCamera.transform.position
                + vrCamera.transform.forward * distance
                + Vector3.up * heightOffset;
            _screenQuad.transform.LookAt(vrCamera.transform);
            _screenQuad.transform.Rotate(0, 180, 0); // 법선 방향 반전
        }

        // 콜라이더 제거 (VR 상호작용 방해 방지)
        Destroy(_screenQuad.GetComponent<Collider>());

        // 머티리얼 설정
        Renderer renderer = _screenQuad.GetComponent<Renderer>();
        Shader shader = Shader.Find("Universal Render Pipeline/Unlit")
                     ?? Shader.Find("Unlit/Texture")
                     ?? Shader.Find("Sprites/Default");
        Material mat = new Material(shader);
        mat.SetColor("_BaseColor", Color.white);
        renderer.material = mat;

        // MJPEGStreamReceiver에 렌더러 연결
        _receiver.targetRenderer = renderer;
    }

void OnDestroy()
    {
        if (_screenQuad != null)
            Destroy(_screenQuad);
    }
}
