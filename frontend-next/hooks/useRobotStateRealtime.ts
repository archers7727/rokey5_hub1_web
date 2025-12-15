'use client'

import { useEffect, useState } from 'react'
import { createClient } from '@/lib/supabase/client'

export function useRobotStateRealtime() {
  const [robotState, setRobotState] = useState<any>(null)
  const supabase = createClient()

  useEffect(() => {
    // Load initial data
    supabase
      .from('robot_state')
      .select('*')
      .eq('id', 'current')
      .single()
      .then(({ data }) => {
        if (data) setRobotState(data)
      })

    // Subscribe to realtime changes
    const channel = supabase
      .channel('robot-state-channel')
      .on(
        'postgres_changes',
        {
          event: 'UPDATE',
          schema: 'public',
          table: 'robot_state',
          filter: 'id=eq.current'
        },
        (payload) => {
          console.log('Robot state updated:', payload.new)
          setRobotState(payload.new)
        }
      )
      .subscribe()

    return () => {
      supabase.removeChannel(channel)
    }
  }, [])

  return robotState
}
