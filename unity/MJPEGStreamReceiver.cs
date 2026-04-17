using System;
using System.Collections.Concurrent;
using System.Net;
using System.IO;
using System.Threading;
using UnityEngine;

public class MJPEGStreamReceiver : MonoBehaviour
{
    [Header("Stream Settings")]
    public string streamUrl = "http://192.168.x.x:8080/stream";
    public Renderer targetRenderer;

    public bool IsConnected { get; private set; } = false;

    private Texture2D _texture;
    private Thread _streamThread;
    private ConcurrentQueue<byte[]> _frameQueue = new ConcurrentQueue<byte[]>();
    private volatile bool _running = false;

    void Start()
    {
        _texture = new Texture2D(2, 2, TextureFormat.RGB24, false);
        StartStream();
    }

    void Update()
    {
        byte[] latestFrame = null;
        while (_frameQueue.TryDequeue(out byte[] frame))
            latestFrame = frame;

        if (latestFrame != null && targetRenderer != null)
        {
            _texture.LoadImage(latestFrame);
            targetRenderer.material.mainTexture = _texture;
            targetRenderer.material.SetTexture("_BaseMap", _texture);
        }
    }

    public void StartStream()
    {
        if (_running) return;
        _running = true;
        _streamThread = new Thread(StreamWorker) { IsBackground = true };
        _streamThread.Start();
    }

    public void StopStream()
    {
        _running = false;
        IsConnected = false;
    }

    private void StreamWorker()
    {
        const int BufferSize = 1024 * 512;
        byte[] buffer = new byte[BufferSize];
        int bufferEnd = 0;

        while (_running)
        {
            try
            {
                HttpWebRequest request = (HttpWebRequest)WebRequest.Create(streamUrl);
                request.Timeout = System.Threading.Timeout.Infinite;
                request.ReadWriteTimeout = System.Threading.Timeout.Infinite;

                using HttpWebResponse response = (HttpWebResponse)request.GetResponse();
                using Stream stream = response.GetResponseStream();

                IsConnected = true;
                Debug.Log($"[MJPEG] Connected to {streamUrl}");
                bufferEnd = 0;

                while (_running)
                {
                    int bytesRead = stream.Read(buffer, bufferEnd, BufferSize - bufferEnd);
                    if (bytesRead == 0) { Debug.LogWarning("[MJPEG] Stream ended."); break; }
                    bufferEnd += bytesRead;

                    int jpegStart = FindBytes(buffer, bufferEnd, 0xFF, 0xD8);
                    int jpegEnd = jpegStart >= 0
                        ? FindBytes(buffer, bufferEnd, 0xFF, 0xD9, jpegStart + 2)
                        : -1;

                    if (jpegStart >= 0 && jpegEnd >= 0)
                    {
                        int length = jpegEnd + 2 - jpegStart;
                        byte[] jpeg = new byte[length];
                        Array.Copy(buffer, jpegStart, jpeg, 0, length);
                        _frameQueue.Enqueue(jpeg);

                        int remaining = bufferEnd - (jpegEnd + 2);
                        Array.Copy(buffer, jpegEnd + 2, buffer, 0, remaining);
                        bufferEnd = remaining;
                    }
                    else if (bufferEnd >= BufferSize - 4096)
                    {
                        bufferEnd = 0;
                    }
                }
            }
            catch (ThreadInterruptedException) { break; }
            catch (Exception e) { Debug.LogWarning($"[MJPEG] {e.Message}. Reconnecting in 1s..."); }

            IsConnected = false;
            if (_running) Thread.Sleep(1000);
        }
    }

    private static int FindBytes(byte[] buf, int length, byte b0, byte b1, int start = 0)
    {
        for (int i = start; i < length - 1; i++)
            if (buf[i] == b0 && buf[i + 1] == b1) return i;
        return -1;
    }

    void OnDestroy()
    {
        StopStream();
        _streamThread?.Interrupt();
        _streamThread?.Join(500);
    }
}
