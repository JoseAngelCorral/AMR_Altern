# Instrucciones para Autenticación en GitHub

## Problema Detectado
Estás logueado como `ZETAOCHO` pero el repositorio es de `JoseAngelCorral`. Necesitas autenticarte correctamente.

## Solución: Usar Token de Acceso Personal

### Paso 1: Crear Token de Acceso Personal

1. Ve a GitHub.com y haz clic en tu perfil (esquina superior derecha)
2. Selecciona **Settings**
3. En el menú lateral, ve a **Developer settings**
4. Selecciona **Personal access tokens** → **Tokens (classic)**
5. Haz clic en **Generate new token** → **Generate new token (classic)**
6. Configura el token:
   - **Note**: "AMR_Altern Push Access" (o cualquier nombre descriptivo)
   - **Expiration**: Elige una duración (90 días, 1 año, o sin expiración)
   - **Scopes**: Marca al menos:
     - ✅ `repo` (acceso completo a repositorios)
7. Haz clic en **Generate token**
8. **IMPORTANTE**: Copia el token inmediatamente (solo se muestra una vez)

### Paso 2: Usar el Token para Push

Tienes dos opciones:

#### Opción A: Usar el token en la URL (temporal)
```bash
git push -u origin https://<TU_TOKEN>@github.com/JoseAngelCorral/AMR_Altern.git main
```

#### Opción B: Configurar Git Credential Manager (recomendado)

1. **Windows - Usar Git Credential Manager:**
```bash
# Configurar para usar el token
git config --global credential.helper manager-core
```

2. Luego cuando hagas push, te pedirá:
   - **Username**: `JoseAngelCorral`
   - **Password**: Pega tu token (no tu contraseña)

#### Opción C: Guardar credenciales en Windows

```bash
# Configurar para guardar credenciales
git config --global credential.helper wincred

# Luego hacer push (te pedirá usuario y token)
git push -u origin main
```

### Paso 3: Verificar que Eres el Propietario

Asegúrate de que:
- El repositorio `AMR_Altern` pertenece a tu cuenta `JoseAngelCorral`
- Tienes permisos de escritura en el repositorio
- Estás logueado en GitHub con la cuenta correcta

## Comandos Rápidos

Una vez que tengas el token:

```bash
# Opción 1: Push directo con token en URL
git push -u origin https://<TU_TOKEN>@github.com/JoseAngelCorral/AMR_Altern.git main

# Opción 2: Configurar y luego push normal
git config --global credential.helper manager-core
git push -u origin main
# Cuando pida credenciales:
# Username: JoseAngelCorral
# Password: <TU_TOKEN>
```

## Alternativa: Usar SSH (más seguro a largo plazo)

Si prefieres usar SSH en lugar de HTTPS:

1. **Generar clave SSH** (si no tienes una):
```bash
ssh-keygen -t ed25519 -C "tu_email@example.com"
```

2. **Agregar clave SSH a GitHub:**
   - Copia el contenido de `~/.ssh/id_ed25519.pub`
   - Ve a GitHub → Settings → SSH and GPG keys → New SSH key
   - Pega la clave y guarda

3. **Cambiar remote a SSH:**
```bash
git remote set-url origin git@github.com:JoseAngelCorral/AMR_Altern.git
git push -u origin main
```

## Verificar Estado Actual

```bash
# Ver remotes configurados
git remote -v

# Ver commits locales
git log --oneline

# Ver estado
git status
```

