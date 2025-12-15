/**
 * 재료(Material) 타입 정의
 */

export type MaterialCategory = 'vegetable' | 'fruit' | 'meat'
export type MaterialSize = 'small' | 'medium' | 'large'

export interface Material {
  id: string
  name: string
  emoji: string
  description: string
  category: MaterialCategory
  sizes: {
    small: number  // cm
    medium: number // cm
    large: number  // cm
  }
}

export interface MaterialSizeOption {
  value: MaterialSize
  label: string
  size: number // cm
}
