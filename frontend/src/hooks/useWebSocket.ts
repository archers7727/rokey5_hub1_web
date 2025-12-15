/**
 * WebSocket 커스텀 훅
 */
import { useEffect, useRef, useState } from 'react'

interface UseWebSocketOptions {
  onMessage?: (data: any) => void
  onOpen?: () => void
  onClose?: () => void
  onError?: (error: Event) => void
}

export function useWebSocket(url: string, options: UseWebSocketOptions = {}) {
  const [isConnected, setIsConnected] = useState(false)
  const [lastMessage, setLastMessage] = useState<any>(null)
  const wsRef = useRef<WebSocket | null>(null)

  useEffect(() => {
    // HTTPS 페이지에서는 wss://, HTTP에서는 ws:// 사용
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:'

    const wsUrl = url.startsWith('ws')
      ? url
      : `${protocol}//${window.location.host}${url}`

    const ws = new WebSocket(wsUrl)
    wsRef.current = ws

    ws.onopen = () => {
      setIsConnected(true)
      options.onOpen?.()
    }

    ws.onmessage = (event) => {
      const data = JSON.parse(event.data)
      setLastMessage(data)
      options.onMessage?.(data)
    }

    ws.onerror = (error) => {
      console.error('WebSocket error:', error)
      options.onError?.(error)
    }

    ws.onclose = () => {
      setIsConnected(false)
      options.onClose?.()
    }

    return () => {
      ws.close()
    }
  }, [url])

  const sendMessage = (data: any) => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify(data))
    }
  }

  return {
    isConnected,
    lastMessage,
    sendMessage,
  }
}
