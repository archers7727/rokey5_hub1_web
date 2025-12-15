'use client'

import { useEffect, useState } from 'react'
import { createClient } from '@/lib/supabase/client'
import type { RealtimeChannel } from '@supabase/supabase-js'

export function useTasksRealtime() {
  const [tasks, setTasks] = useState<any[]>([])
  const supabase = createClient()

  useEffect(() => {
    // Load initial data
    supabase
      .from('tasks')
      .select('*')
      .order('created_at', { ascending: false })
      .then((response: any) => {
        if (response.data) setTasks(response.data)
      })

    // Subscribe to realtime changes
    const channel: RealtimeChannel = supabase
      .channel('tasks-channel')
      .on(
        'postgres_changes',
        {
          event: '*', // Subscribe to INSERT, UPDATE, DELETE
          schema: 'public',
          table: 'tasks'
        },
        (payload) => {
          console.log('Task changed:', payload)

          if (payload.eventType === 'INSERT') {
            setTasks(prev => [payload.new, ...prev])
          } else if (payload.eventType === 'UPDATE') {
            setTasks(prev =>
              prev.map(task =>
                task.id === payload.new.id ? payload.new : task
              )
            )
          } else if (payload.eventType === 'DELETE') {
            setTasks(prev =>
              prev.filter(task => task.id !== payload.old.id)
            )
          }
        }
      )
      .subscribe()

    return () => {
      supabase.removeChannel(channel)
    }
  }, [])

  return tasks
}
